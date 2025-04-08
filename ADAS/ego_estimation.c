#include <math.h>
#include <string.h>
#include <stdio.h>
#include "ego_estimation.h"

/* 스파이크(노이즈) 체크 함수 */
static bool CheckSpike(float newVal, float oldVal, float threshold)
{
    float diff = newVal - oldVal;
    return (fabsf(diff) > threshold);
}

/* 칼만 필터 예측 & 업데이트 등은 간단 예시 */
void EgoEstimation_Init(EgoEstimationKFState_t *pState)
{
    if(!pState) return;
    memset(pState, 0, sizeof(EgoEstimationKFState_t));
    /* 초기 공분산 등 필요시 설정 */
}

void EgoEstimation_Update(const TimeData_t *timeData,
                          const GPSData_t *gpsData,
                          const IMUData_t *imuData,
                          EgoData_t       *pEgoData,
                          EgoEstimationKFState_t *pState)
{
    if(!timeData || !gpsData || !imuData || !pEgoData || !pState) return;

    /* 1) GPS 유효성 판단 */
    float gps_dt = fabsf(timeData->Current_Time - gpsData->GPS_Timestamp);
    bool gps_valid = (gps_dt <= GPS_VALID_TIME_THRESH);

    /* 2) 스파이크 제거 */
    float accel_x = imuData->Linear_Acceleration_X;
    float accel_y = imuData->Linear_Acceleration_Y;
    float yawRate = imuData->Yaw_Rate;
    float gps_vx  = gpsData->GPS_Velocity_X;
    float gps_vy  = gpsData->GPS_Velocity_Y;

    if(CheckSpike(accel_x, pState->Prev_Accel_X, MAX_SENSOR_NOISE_ACCEL)) {
        accel_x = pState->Prev_Accel_X;
    }
    if(CheckSpike(accel_y, pState->Prev_Accel_Y, MAX_SENSOR_NOISE_ACCEL)) {
        accel_y = pState->Prev_Accel_Y;
    }
    if(CheckSpike(yawRate, pState->Prev_YawRate, MAX_SENSOR_NOISE_YAWRATE)) {
        yawRate = pState->Prev_YawRate;
    }
    if(CheckSpike(gps_vx, pState->Prev_GPS_VelX, MAX_SENSOR_NOISE_GPSVEL) ||
       CheckSpike(gps_vy, pState->Prev_GPS_VelY, MAX_SENSOR_NOISE_GPSVEL)){
        gps_valid = false;
    }

    /* 저장 */
    pState->Prev_Accel_X = accel_x;
    pState->Prev_Accel_Y = accel_y;
    pState->Prev_YawRate = yawRate;
    if(gps_valid){
        pState->Prev_GPS_VelX = gps_vx;
        pState->Prev_GPS_VelY = gps_vy;
    }

    /* 3) delta_t 계산 */
    float delta_t = (timeData->Current_Time - pState->Prev_Update_Time) * 0.001f; /* ms->sec */
    if(delta_t < 0.0f) delta_t = 0.01f; 
    pState->Prev_Update_Time = timeData->Current_Time;

    /* 4) 칼만 필터 예측(간단) */
    /* state = [vx, vy, ax, ay, heading]
       예) vx_k = vx_(k-1) + ax*(dt)
    */
    float vx  = pState->state[0];
    float vy  = pState->state[1];
    float ax_ = pState->state[2];
    float ay_ = pState->state[3];
    float hdg = pState->state[4];

    /* 예측 */
    vx  += accel_x * delta_t;
    vy  += accel_y * delta_t;
    hdg += yawRate * delta_t; /* deg/s * s -> deg */

    /* 5) 관측 업데이트 (GPS) */
    if(gps_valid){
        vx = 0.7f*vx + 0.3f*gps_vx; 
        vy = 0.7f*vy + 0.3f*gps_vy;
    }

    /* 저장 back */
    pState->state[0] = vx;
    pState->state[1] = vy;
    pState->state[2] = accel_x;
    pState->state[3] = accel_y;
    pState->state[4] = hdg;

    /* 6) 결과를 EgoData에 반영 */
    pEgoData->Ego_Velocity_X     = vx;
    pEgoData->Ego_Velocity_Y     = vy;
    pEgoData->Ego_Acceleration_X = accel_x;
    pEgoData->Ego_Acceleration_Y = accel_y;
    pEgoData->Ego_Heading        = hdg; 
    pEgoData->Ego_Yaw_Rate       = yawRate;

    /* Position은 (0,0,0) 유지. 
       실제론 적분해서 추정 가능. */
    pEgoData->Ego_Position_X = 0.0f;
    pEgoData->Ego_Position_Y = 0.0f;
    pEgoData->Ego_Position_Z = 0.0f;
}
