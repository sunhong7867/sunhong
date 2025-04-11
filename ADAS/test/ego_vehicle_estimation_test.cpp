#include <gtest/gtest.h>
#include <iostream>
#include "ego_vehicle_estimation.h"
#include "adas_shared.h"

// Test Fixture Class
class EgoVehicleEstimationTest : public ::testing::Test {
protected:
    TimeData_t timeData;
    GPSData_t gpsData;
    IMUData_t imuData;
    EgoData_t egoData;
    EgoVehicleKFState_t kfState;

    void SetUp() override {
        timeData.Current_Time = 0.0f;

        gpsData.GPS_Velocity_X = 0.0f;
        gpsData.GPS_Velocity_Y = 0.0f;
        gpsData.GPS_Timestamp = 0.0f;

        imuData.Linear_Acceleration_X = 0.0f;
        imuData.Linear_Acceleration_Y = 0.0f;
        imuData.Yaw_Rate = 0.0f;

        InitEgoVehicleKFState(&kfState);
        kfState.Previous_Update_Time = 0.0f;
    }

    void TearDown() override {}
};

// TC_EGO_001: 정상 동작 검증
TEST_F(EgoVehicleEstimationTest, TC_EGO_001) {
    kfState.X[0] = 10.0f; kfState.X[1] = 0.0f; kfState.X[2] = 1.0f; kfState.X[3] = 0.5f; kfState.X[4] = 0.0f;
    kfState.Previous_Update_Time = 900.0f;

    timeData.Current_Time = 1000.0f;

    gpsData.GPS_Velocity_X = 10.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    gpsData.GPS_Timestamp = 995.0f;

    imuData.Linear_Acceleration_X = 1.0f;
    imuData.Linear_Acceleration_Y = 0.5f;
    imuData.Yaw_Rate = 5.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    float exp_vx = 10.1f;
    float exp_vy = 0.05f;
    float exp_ax = 2.0f;
    float exp_ay = 1.0f;
    float exp_heading = 0.5f;

    std::cout << "[TC_EGO_001] Expected Vx=" << exp_vx << ", Vy=" << exp_vy
              << ", Ax=" << exp_ax << ", Ay=" << exp_ay
              << ", Heading=" << exp_heading << "\n";
    std::cout << "             Actual   Vx=" << egoData.Ego_Velocity_X << ", Vy=" << egoData.Ego_Velocity_Y
              << ", Ax=" << egoData.Ego_Acceleration_X << ", Ay=" << egoData.Ego_Acceleration_Y
              << ", Heading=" << egoData.Ego_Heading << "\n";

    EXPECT_NEAR(egoData.Ego_Velocity_X, exp_vx, 0.11f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, exp_vy, 0.1f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, exp_ax, 0.1f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, exp_ay, 0.1f);
    EXPECT_NEAR(egoData.Ego_Heading, exp_heading, 0.1f);
}

// TC_EGO_002: GPS 무효/스파이크 검출
TEST_F(EgoVehicleEstimationTest, TC_EGO_002) {
    kfState.X[0] = 12.0f; kfState.X[1] = 1.0f; kfState.X[2] = 0.0f; kfState.X[3] = 0.0f; kfState.X[4] = 10.0f;
    kfState.Previous_Update_Time = 1900.0f;

    timeData.Current_Time = 2000.0f;

    gpsData.GPS_Velocity_X = 30.0f;
    gpsData.GPS_Velocity_Y = -20.0f;
    gpsData.GPS_Timestamp = 1500.0f;

    imuData.Linear_Acceleration_X = 0.8f;
    imuData.Linear_Acceleration_Y = -0.3f;
    imuData.Yaw_Rate = 2.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    float exp_vx = 12.0f;
    float exp_vy = 1.0f;
    float exp_ax = 0.8f;
    float exp_ay = -0.3f;
    float exp_heading = 10.2f;

    std::cout << "[TC_EGO_002] Expected Vx=" << exp_vx << ", Vy=" << exp_vy
              << ", Ax=" << exp_ax << ", Ay=" << exp_ay
              << ", Heading=" << exp_heading << "\n";
    std::cout << "             Actual   Vx=" << egoData.Ego_Velocity_X << ", Vy=" << egoData.Ego_Velocity_Y
              << ", Ax=" << egoData.Ego_Acceleration_X << ", Ay=" << egoData.Ego_Acceleration_Y
              << ", Heading=" << egoData.Ego_Heading << "\n";

    EXPECT_NEAR(egoData.Ego_Velocity_X, exp_vx, 0.1f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, exp_vy, 0.1f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, exp_ax, 0.1f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, exp_ay, 0.1f);
    EXPECT_NEAR(egoData.Ego_Heading, exp_heading, 0.1f);
}

// TC_EGO_003: IMU 스파이크 보정 검증
TEST_F(EgoVehicleEstimationTest, TC_EGO_003) {
    kfState.X[0] = 10.0f; kfState.X[1] = 0.0f; kfState.X[2] = 1.0f; kfState.X[3] = 0.0f; kfState.X[4] = 0.0f;
    kfState.Previous_Update_Time = 1400.0f;

    timeData.Current_Time = 1500.0f;

    gpsData.GPS_Velocity_X = 12.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    gpsData.GPS_Timestamp = 1490.0f;

    imuData.Linear_Acceleration_X = 5.0f;
    imuData.Linear_Acceleration_Y = 0.0f;
    imuData.Yaw_Rate = 100.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    float exp_vx = 10.1f;
    float exp_vy = 0.0f;
    float exp_ax = 1.0f;
    float exp_ay = 0.0f;
    float exp_heading = 0.0f;

    std::cout << "[TC_EGO_003] Expected Vx=" << exp_vx << ", Vy=" << exp_vy
              << ", Ax=" << exp_ax << ", Ay=" << exp_ay
              << ", Heading=" << exp_heading << "\n";
    std::cout << "             Actual   Vx=" << egoData.Ego_Velocity_X << ", Vy=" << egoData.Ego_Velocity_Y
              << ", Ax=" << egoData.Ego_Acceleration_X << ", Ay=" << egoData.Ego_Acceleration_Y
              << ", Heading=" << egoData.Ego_Heading << "\n";

    EXPECT_NEAR(egoData.Ego_Velocity_X, exp_vx, 0.1f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, exp_vy, 0.1f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, exp_ax, 0.1f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, exp_ay, 0.1f);
    EXPECT_NEAR(egoData.Ego_Heading, exp_heading, 0.1f);
}

// TC_EGO_004: Heading 정규화 검증
TEST_F(EgoVehicleEstimationTest, TC_EGO_004) {
    kfState.X[0] = 5.0f; kfState.X[1] = 0.0f; kfState.X[2] = 0.0f; kfState.X[3] = 0.0f; kfState.X[4] = 170.0f;
    kfState.Previous_Update_Time = 1100.0f;

    timeData.Current_Time = 1200.0f;

    gpsData.GPS_Velocity_X = 15.0f;
    gpsData.GPS_Velocity_Y = 0.0f;
    gpsData.GPS_Timestamp = 1190.0f;

    imuData.Linear_Acceleration_X = 0.0f;
    imuData.Linear_Acceleration_Y = 0.2f;
    imuData.Yaw_Rate = 20.0f;

    EgoVehicleEstimation(&timeData, &gpsData, &imuData, &egoData, &kfState);

    float exp_vx = 5.0f;
    float exp_vy = 0.0f;
    float exp_ax = 0.0f;
    float exp_ay = 0.2f;
    float exp_heading = 172.0f;

    std::cout << "[TC_EGO_004] Expected Vx=" << exp_vx << ", Vy=" << exp_vy
              << ", Ax=" << exp_ax << ", Ay=" << exp_ay
              << ", Heading=" << exp_heading << "\n";
    std::cout << "             Actual   Vx=" << egoData.Ego_Velocity_X << ", Vy=" << egoData.Ego_Velocity_Y
              << ", Ax=" << egoData.Ego_Acceleration_X << ", Ay=" << egoData.Ego_Acceleration_Y
              << ", Heading=" << egoData.Ego_Heading << "\n";

    EXPECT_NEAR(egoData.Ego_Velocity_X, exp_vx, 0.1f);
    EXPECT_NEAR(egoData.Ego_Velocity_Y, exp_vy, 0.1f);
    EXPECT_NEAR(egoData.Ego_Acceleration_X, exp_ax, 0.1f);
    EXPECT_NEAR(egoData.Ego_Acceleration_Y, exp_ay, 0.1f);
    EXPECT_NEAR(egoData.Ego_Heading, exp_heading, 0.1f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}