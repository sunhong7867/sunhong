// ego_vehicle_estimation_test.cpp

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
  #include "ego_vehicle_estimation.h"
  #include "adas_shared.h"
}

// 헤더에 선언된 초기화 함수를 사용하기 위한 매크로
#define INIT_KF_STATE(state_ptr) InitEgoVehicleKFState(state_ptr)

//
// Test 1: 좌표 기준 고정 검증
// 요구사항: Ego 차량의 좌표는 항상 (0, 0, 0) 으로 고정되어야 한다.
//
TEST(EgoVehicleEstimationTest, CoordinateFixedTest) {
    TimeData_t timeData = {100.0f};              // 제어 주기 시각 100ms
    GPSData_t gpsData   = {20.0f, 0.0f, 95.0f};    // GPS: 속도 20, 타임스탬프 95ms (차이 5ms, 유효)
    IMUData_t imuData   = {0.5f, 0.1f, 2.0f};       // IMU: accel 및 yaw 값
    EgoData_t egoData;
    EgoVehicleKFState_t kfState;
    INIT_KF_STATE(&kfState);
    
    // 좌표 고정 요구사항: Ego_Position은 (0,0,0)
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    
    EXPECT_NEAR(egoData.Ego_Position_X, 0.0f, 0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Y, 0.0f, 0.001f);
    EXPECT_NEAR(egoData.Ego_Position_Z, 0.0f, 0.001f);
}

//
// Test 2: 정상 센서 입력 (GPS 업데이트 활성)
// 요구사항: GPS 센서 데이터가 50ms 이하 차이인 경우 유효하게 반영되어,
// 칼만 필터 업데이트 단계에서 IMU 예측값과 결합되어 보정된 값이 산출되어야 한다.
//
TEST(EgoVehicleEstimationTest, NormalSensorUpdate) {
    TimeData_t timeData = {100.0f};              // 제어 루프 시각 100ms
    // GPS: 타임스탬프 95ms → |100-95| = 5ms (<50ms) → 유효
    GPSData_t gpsData = {20.0f, 0.0f, 95.0f};
    IMUData_t imuData = {0.5f, 0.1f, 2.0f};        // IMU 데이터
    EgoData_t egoData;
    EgoVehicleKFState_t kfState;
    INIT_KF_STATE(&kfState);
    
    // 정상적인 GPS 업데이트를 위해, 이전 값을 현재 GPS 값으로 초기화
    kfState.Prev_GPS_Vel_X = gpsData.GPS_Velocity_X;
    kfState.Prev_GPS_Vel_Y = gpsData.GPS_Velocity_Y;
    
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    
    /*  
       예상 계산 (칼만 필터 가정):
         - delta_t = 100 - 0 = 100 (초기 Previous_Update_Time가 0)
         - 예측 단계: 
             vx_pred = 0 + (0.5 * 100) = 50,
             vy_pred = 0 + (0.1 * 100) = 10,
             heading_pred = 0 + (2.0 * 100) = 200
         - GPS 업데이트 (alpha = 0.3 가정):
             vx = 0.7 * 50 + 0.3 * 20 = 35 + 6 = 41,
             vy = 0.7 * 10 + 0.3 * 0  = 7
         - heading: 200 > 180 → 200 - 360 = -160
    */
    EXPECT_NEAR(egoData.Ego_Velocity_X, 41.0f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, 7.0f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 0.5f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, 0.1f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Heading, -160.0f, 0.01f);
}

//
// Test 3: GPS 업데이트 비활성화 (GPS 시간 오차)
// 요구사항: GPS 시간 차이가 50ms를 초과하면, GPS 데이터는 무효로 간주하고,
// 이전 루프의 유효 GPS 속도 값을 사용해야 한다.
//
TEST(EgoVehicleEstimationTest, GPSUpdateDisabled) {
    TimeData_t timeData = {100.0f};
    // GPS: 타임스탬프 40ms → |100-40| = 60ms (>50ms) → 업데이트 비활성화
    GPSData_t gpsData = {20.0f, 0.0f, 40.0f};
    IMUData_t imuData = {0.5f, 0.1f, 2.0f};
    EgoData_t egoData;
    EgoVehicleKFState_t kfState;
    INIT_KF_STATE(&kfState);
    
    // 이 테스트에서는 이전 GPS 값이 0인 상태를 그대로 둡니다.
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    
    // 예상: GPS 업데이트가 비활성화되어, 예측 단계의 값이 그대로 출력됨:
    // vx = 0.5 * 100 = 50, vy = 0.1 * 100 = 10, heading 예측: 2.0*100=200 → 정규화하면 -160.
    EXPECT_NEAR(egoData.Ego_Velocity_X, 50.0f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, 10.0f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 0.5f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, 0.1f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Heading, -160.0f, 0.01f);
}

//
// Test 4: 센서 스파이크 처리 (IMU 이상값)
// 요구사항: IMU의 선형 가속도가 임계치(±3.0 m/s²)를 초과하는 경우, 
// 이전 정상값으로 대체해야 한다.
//
TEST(EgoVehicleEstimationTest, SensorSpikeHandling) {
    TimeData_t timeData = {10.0f};               // 제어 루프 시각 10ms
    // 유효한 GPS: 타임스탬프 9ms → 차이가 1ms (<50ms)
    GPSData_t gpsData = {20.0f, 0.0f, 9.0f};
    // IMU: accel_x가 4.0f (임계치 3.0 초과 → 스파이크로 간주되어 이전 값 (0)을 사용)
    IMUData_t imuData = {4.0f, 0.0f, 2.0f};
    EgoData_t egoData;
    EgoVehicleKFState_t kfState;
    INIT_KF_STATE(&kfState);
    // 정상적인 GPS 보정을 위해, 이전 GPS 속도를 현재 값으로 초기화
    kfState.Prev_GPS_Vel_X = gpsData.GPS_Velocity_X;
    kfState.Prev_GPS_Vel_Y = gpsData.GPS_Velocity_Y;
    
    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);
    
    /*  
       예상 계산:
         - delta_t = 10 - 0 = 10
         - accel spike 처리: 4.0f에서 급격한 변화가 감지되어, raw_accel_x 대신 이전 값 0 사용.
         - 예측: vx = 0 + 0*10 = 0, vy = 0*10 = 0, heading = 2.0*10 = 20
         - GPS 업데이트 (alpha=0.3):
              vx = 0.7 * 0 + 0.3 * 20 = 6.0,
              vy = 0.7 * 0 + 0.3 * 0 = 0
    */
    EXPECT_NEAR(egoData.Ego_Velocity_X, 6.0f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, 0.0f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, 0.0f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, 0.0f, 0.01f);
    EXPECT_NEAR(egoData.Ego_Heading, 20.0f, 0.01f);
}

//
// main 함수: Google Test 실행
//
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
