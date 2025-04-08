#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ego_vehicle_estimation.h"

/* 
  ───────────────
   예: adas_shared.h (또는 해당 설계서 기반 헤더)에 이미 정의된 자료형
   - TimeData_t      { float Current_Time; }
   - GPSData_t       { float GPS_Velocity_X, GPS_Velocity_Y, GPS_Timestamp; }
   - IMUData_t       { float Linear_Acceleration_X, Linear_Acceleration_Y, Yaw_Rate; }
   - EgoData_t       { float Ego_Velocity_X, Ego_Velocity_Y, Ego_Acceleration_X, ... }
   - etc.
  ───────────────
*/

/*=== 내부: 칼만 필터 상태 정의 (설계서상 "x̂, P" 등) ===*/
typedef struct {
    /* 지난 루프에서 보정된 상태벡터 [vx, vy, ax, ay, heading], 등 */
    float X[5];  /* x̂ */
    float P[25]; /* 공분산행렬 5x5 */

    /* 2.2.2.2 절차에 따른 내부 변수 */
    float Last_GPS_Velocity_X; 
    float Last_GPS_Velocity_Y;
    float Last_GPS_Timestamp;
    float Previous_Update_Time;

    /* 노이즈 제거를 위한 이전 정상값 */
    float Prev_Accel_X;
    float Prev_Accel_Y;
    float Prev_Yaw_Rate;
    float Prev_GPS_Vel_X;
    float Prev_GPS_Vel_Y;
} EgoVehicleKFState_t;

/*=== 임계값 상수들 (설계서 내 "±3.0 m/s²", "±30 °/s", etc.) ===*/
#define GPS_VALID_TIME_MS   50.0f
#define ACCEL_SPIKE_THRESH  3.0f
#define YAW_SPIKE_THRESH    30.0f
#define GPS_VEL_SPIKE_THRESH 10.0f

/*=== 칼만 필터 설계 파라미터 예시 ===*/
#define Q_process 0.01f   /* 프로세스 노이즈 (단순 예) */
#define R_gps     0.1f    /* 관측 노이즈 (단순 예) */

/*=== 스파이크(노이즈) 감지 함수 ===*/
static bool CheckSpike(float newVal, float oldVal, float threshold)
{
    float diff = newVal - oldVal;
    return (fabsf(diff) > threshold);
}

/* 
설계서 "2.2.2.1.1" : EgoVehicleEstimation()
- IMU + GPS 데이터를 융합하여 Ego 차량 속도,가속도, Heading 추정
*/

void EgoVehicleEstimation(
    /* 입력 */
    const TimeData_t        *timeData,         /* Current_Time */
    const GPSData_t         *gpsData,          /* GPS_Velocity_X, GPS_Velocity_Y, GPS_Timestamp */
    const IMUData_t         *imuData,          /* Linear_Acceleration_X, Y, Yaw_Rate */
    /* 출력 */
    EgoData_t               *egoData,          /* Ego_Velocity_X, Y, Accel_X, Y ... */
    /* 내부 KF 상태 */
    EgoVehicleKFState_t     *kfState           /* 포함: Last_GPS_Velocity_X, Filtered_Accel_X 등등 */
)
{
    /* 1) 좌표 기준 고정:
       - 설계서상 "Ego_Position은 항상 (0,0,0)" 
         => 여기서 바로 세팅 */
    egoData->Ego_Position_X = 0.0f;
    egoData->Ego_Position_Y = 0.0f;
    egoData->Ego_Position_Z = 0.0f;

    /*---------------------------
     * 2) 센서 데이터 동기화 & GPS 유효성 
     *---------------------------*/
    float gps_dt = fabsf(timeData->Current_Time - gpsData->GPS_Timestamp);
    bool gps_update_enabled = true; /* 초기값 */

    if(gps_dt > GPS_VALID_TIME_MS) {
        /* 유효 범위 초과 → 이전 GPS 속도를 사용 */
        gps_update_enabled = false;
    }

    /* delta_t 계산 */
    float delta_t = timeData->Current_Time - kfState->Previous_Update_Time;
    if(delta_t < 0.0f) {
        delta_t = 0.01f; /* 혹시 음수면 최소값 보정 */
    }
    kfState->Previous_Update_Time = timeData->Current_Time;

    /*---------------------------
     * 3) 노이즈 제거 (스파이크 등) & 드리프트 보정 준비
     *---------------------------*/
    /* 현재 센서 원시값 */
    float raw_accel_x = imuData->Linear_Acceleration_X;
    float raw_accel_y = imuData->Linear_Acceleration_Y;
    float raw_yawRate = imuData->Yaw_Rate;

    float raw_gps_vx  = gpsData->GPS_Velocity_X;
    float raw_gps_vy  = gpsData->GPS_Velocity_Y;

    /* 3.2 스파이크 제거 */
    if(CheckSpike(raw_accel_x, kfState->Prev_Accel_X, ACCEL_SPIKE_THRESH)) {
        /* 노이즈 → 이전값 사용 */
        raw_accel_x = kfState->Prev_Accel_X;
    }
    if(CheckSpike(raw_accel_y, kfState->Prev_Accel_Y, ACCEL_SPIKE_THRESH)) {
        raw_accel_y = kfState->Prev_Accel_Y;
    }
    if(CheckSpike(raw_yawRate, kfState->Prev_Yaw_Rate, YAW_SPIKE_THRESH)) {
        raw_yawRate = kfState->Prev_Yaw_Rate;
    }

    /* GPS */
    if(CheckSpike(raw_gps_vx, kfState->Prev_GPS_Vel_X, GPS_VEL_SPIKE_THRESH) ||
       CheckSpike(raw_gps_vy, kfState->Prev_GPS_Vel_Y, GPS_VEL_SPIKE_THRESH))
    {
        /* GPS가 노이즈로 보임 → 이번 루프 업데이트 불가 */
        gps_update_enabled = false;
    }

    /* 이전값 갱신 */
    kfState->Prev_Accel_X   = raw_accel_x;
    kfState->Prev_Accel_Y   = raw_accel_y;
    kfState->Prev_Yaw_Rate  = raw_yawRate;
    if(gps_update_enabled) {
        kfState->Prev_GPS_Vel_X = raw_gps_vx;
        kfState->Prev_GPS_Vel_Y = raw_gps_vy;
    }

    /*---------------------------
     * 4) 예측 단계 (칼만 필터)
     *---------------------------*/
    /* 상태벡터 x̂ = [vx, vy, ax, ay, heading], 일단 간단 예 */
    float vx = kfState->X[0];
    float vy = kfState->X[1];
    float ax = kfState->X[2];
    float ay = kfState->X[3];
    float hdg= kfState->X[4];

    /* 예측: (간단화) vx += ax*dt, hdg += yawRate*dt ... */
    vx  += raw_accel_x * delta_t;
    vy  += raw_accel_y * delta_t;
    hdg += raw_yawRate * delta_t;

    /* 공분산 P 예측 (간단화) */
    /* 실제론: P = A P A^T + Q => 여기선 생략 or 약식 */

    /* 임시저장 */
    float pred_vx=vx, pred_vy=vy, pred_hdg=hdg;

    /*---------------------------
     * 5) 업데이트 단계 (GPS 관측)
     *---------------------------*/
    if(gps_update_enabled) {
        /* 관측(Filtered_GPS_Vel_X, Y) = raw_gps_vx, raw_gps_vy */
        /* y = z - Hx => z= [gps_vx, gps_vy], H=?? => 일단 속도만 관측 */
        float obs_vx = raw_gps_vx;
        float obs_vy = raw_gps_vy;

        /* 칼만 이득 (간단화) */
        /* ... */

        /* 보정: vx, vy 만 보정 */
        float alpha= 0.3f; /* 임의, 실제는 K */
        vx= (1.0f - alpha)*pred_vx + alpha*obs_vx;
        vy= (1.0f - alpha)*pred_vy + alpha*obs_vy;

        /* 공분산 P 보정 (생략) */
    } 
    /* else => GPS 미갱신, 예측값 그대로 */

    /* heading, ax, ay는 그대로 예측값 */
    ax = raw_accel_x;
    ay = raw_accel_y;
    hdg= pred_hdg;

    /* 정규화: heading은 [-180,180] */
    while(hdg> 180.0f)  hdg -= 360.0f;
    while(hdg<-180.0f)  hdg += 360.0f;

    /* 최종 상태에 반영 */
    kfState->X[0]= vx; 
    kfState->X[1]= vy;
    kfState->X[2]= ax;
    kfState->X[3]= ay;
    kfState->X[4]= hdg;

    /*---------------------------
     * 최종 출력: EgoData
     *---------------------------*/
    egoData->Ego_Velocity_X      = vx;
    egoData->Ego_Velocity_Y      = vy;
    egoData->Ego_Acceleration_X  = ax;
    egoData->Ego_Acceleration_Y  = ay;
    egoData->Ego_Heading         = hdg;

    /* 설계서상 "Ego_Position=(0,0,0) 고정" → 이미 위에서 세팅 */
}

/*=========================
 초기화 함수 예시
=========================*/
void InitEgoVehicleKFState(EgoVehicleKFState_t *kfState)
{
    if(!kfState) return;
    memset(kfState,0,sizeof(*kfState));
    /* 필요시 P 초기값 등 세팅 */
    kfState->X[0]=0.0f; /* vx */
    kfState->X[1]=0.0f; /* vy */
    kfState->X[2]=0.0f; /* ax */
    kfState->X[3]=0.0f; /* ay */
    kfState->X[4]=0.0f; /* heading */

    /* P, etc. */
}
