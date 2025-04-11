#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ego_vehicle_estimation.h"

/*───────────────────────────── 
  예: adas_shared.h (또는 해당 설계서 기반 헤더)에 이미 정의된 자료형
  - TimeData_t      { float Current_Time; }
  - GPSData_t       { float GPS_Velocity_X, GPS_Velocity_Y, GPS_Timestamp; }
  - IMUData_t       { float Linear_Acceleration_X, Linear_Acceleration_Y, Yaw_Rate; }
  - EgoData_t       { float Ego_Velocity_X, Ego_Velocity_Y, Ego_Acceleration_X, Ego_Acceleration_Y, Ego_Heading, Ego_Position_X, Ego_Position_Y, Ego_Position_Z; }
  - etc.
─────────────────────────────*/

/* 임계값 상수 (설계서 상 스파이크/유효성 기준) */
#define GPS_VALID_TIME_MS       50.0f
#define ACCEL_SPIKE_THRESH      3.0f
#define YAW_SPIKE_THRESH        30.0f
#define GPS_VEL_SPIKE_THRESH    10.0f

/* 칼만 필터 설계 파라미터 */
#define Q_PROCESS       0.01f   /* 프로세스 노이즈 (대각 성분) */
#define R_GPS           0.1f    /* 관측 노이즈 (대각 성분) */

/* 2x2 행렬의 역행렬 계산 (2x2 matrix inversion) */
static bool Invert2x2(const float S[4], float S_inv[4])
{
    float det = S[0] * S[3] - S[1] * S[2];
    if(fabsf(det) < 1e-6) return false;
    float invDet = 1.0f / det;
    S_inv[0] =  S[3] * invDet;
    S_inv[1] = -S[1] * invDet;
    S_inv[2] = -S[2] * invDet;
    S_inv[3] =  S[0] * invDet;
    return true;
}

/* 스파이크(노이즈) 감지 함수 */
static bool CheckSpike(float newVal, float oldVal, float threshold)
{
    float diff = newVal - oldVal;
    return (fabsf(diff) > threshold);
}

/*─────────────────────────────────────────
  EgoVehicleEstimation()
  - IMU + GPS 센서 데이터를 융합하여 Ego 차량의 속도, 가속도, Heading 추정
─────────────────────────────────────────*/
void EgoVehicleEstimation(
    /* 입력 */
    const TimeData_t        *timeData,         /* Current_Time (ms) */
    const GPSData_t         *gpsData,          /* GPS_Velocity_X, GPS_Velocity_Y, GPS_Timestamp (ms) */
    const IMUData_t         *imuData,          /* Linear_Acceleration_X, Linear_Acceleration_Y, Yaw_Rate */
    /* 출력 */
    EgoData_t               *egoData,          /* Ego_Velocity, Acceleration, Heading, Position 등 */
    /* 내부 KF 상태 */
    EgoVehicleKFState_t     *kfState           /* 내부 변수들 및 칼만 필터 상태 (X, P 등) */
)
{
    /*───────────────────────────── 
      1) 좌표 기준 고정: Ego_Position = (0,0,0)
    ─────────────────────────────*/
    egoData->Ego_Position_X = 0.0f;
    egoData->Ego_Position_Y = 0.0f;
    egoData->Ego_Position_Z = 0.0f;

    /*───────────────────────────── 
      2) 센서 데이터 동기화 및 GPS 유효성 판단
         - gps_dt 계산, 허용 오차(GPS_VALID_TIME_MS) 초과 시 이전 GPS 값 사용
    ─────────────────────────────*/
    float gps_dt = fabsf(timeData->Current_Time - gpsData->GPS_Timestamp);
    bool gps_update_enabled = (gps_dt <= GPS_VALID_TIME_MS);

    /* delta_t 계산 (현재 루프 시간 - 이전 업데이트 시간) */
    float delta_t = timeData->Current_Time - kfState->Previous_Update_Time;
    if(delta_t <= 0.0f) {
        delta_t = 0.01f;  /* 최소 시간 간격 보정 */
    }
    kfState->Previous_Update_Time = timeData->Current_Time;

    /*───────────────────────────── 
      3) 노이즈 제거 & 드리프트 보정 준비
         - IMU 및 GPS 센서의 스파이크 제거
    ─────────────────────────────*/
    float raw_accel_x = imuData->Linear_Acceleration_X;
    float raw_accel_y = imuData->Linear_Acceleration_Y;
    float raw_yawRate = imuData->Yaw_Rate;

    float raw_gps_vx  = gpsData->GPS_Velocity_X;
    float raw_gps_vy  = gpsData->GPS_Velocity_Y;

    /* IMU 스파이크 제거 */
    if(CheckSpike(raw_accel_x, kfState->Prev_Accel_X, ACCEL_SPIKE_THRESH)) {
        raw_accel_x = kfState->Prev_Accel_X;
    }
    if(CheckSpike(raw_accel_y, kfState->Prev_Accel_Y, ACCEL_SPIKE_THRESH)) {
        raw_accel_y = kfState->Prev_Accel_Y;
    }
    if(CheckSpike(raw_yawRate, kfState->Prev_Yaw_Rate, YAW_SPIKE_THRESH)) {
        raw_yawRate = kfState->Prev_Yaw_Rate;
    }

    /* GPS 스파이크 제거 */
    if(CheckSpike(raw_gps_vx, kfState->Prev_GPS_Vel_X, GPS_VEL_SPIKE_THRESH) ||
       CheckSpike(raw_gps_vy, kfState->Prev_GPS_Vel_Y, GPS_VEL_SPIKE_THRESH)) {
        gps_update_enabled = false;  /* 이번 루프는 GPS 업데이트 생략 */
    }

    /* 이전 센서값 갱신 */
    kfState->Prev_Accel_X  = raw_accel_x;
    kfState->Prev_Accel_Y  = raw_accel_y;
    kfState->Prev_Yaw_Rate = raw_yawRate;
    if(gps_update_enabled) {
        kfState->Prev_GPS_Vel_X = raw_gps_vx;
        kfState->Prev_GPS_Vel_Y = raw_gps_vy;
    }

    /*───────────────────────────── 
      4) 칼만 필터 예측 단계
         - 상태 예측: x̂ = A * X_prev + B * u, 
           여기서 u = [raw_accel_x, raw_accel_y, raw_yawRate]^T
         - 공분산 예측: P = A * P * A^T + Q
    ─────────────────────────────*/
    /* 상태 벡터 차원: 5 */
    float X_pred[5] = {0};
    float A[25] = {
         1.0f, 0.0f, delta_t, 0.0f,   0.0f,
         0.0f, 1.0f, 0.0f,    delta_t, 0.0f,
         0.0f, 0.0f, 1.0f,    0.0f,    0.0f,
         0.0f, 0.0f, 0.0f,    1.0f,    0.0f,
         0.0f, 0.0f, 0.0f,    0.0f,    1.0f
    };

    float u[3] = { raw_accel_x, raw_accel_y, raw_yawRate };

    /* X_pred = A * X + B * u */
    X_pred[0] = kfState->X[0] + (delta_t / 1000.0f) * kfState->X[2];  // 예측된 속도
    X_pred[1] = kfState->X[1] + (delta_t / 1000.0f) * kfState->X[3];  // 예측된 속도
    X_pred[2] = kfState->X[2] + u[0];  // 직접 IMU 측정값 반영
    X_pred[3] = kfState->X[3] + u[1];
    X_pred[4] = kfState->X[4] + (delta_t / 1000.0f) * u[2];  // 예측된 헤딩

    /* 공분산 예측: P_pred = A * P * A^T + Q */
    float P_pred[25] = {0};
    float M[25] = {0};  

    /* M = A * P */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) {
                sum += A[i*5+k] * kfState->P[k*5+j];
            }
            M[i*5+j] = sum;
        }
    }
    /* P_pred = M * A^T + Q, A^T[j*5 + i] = A[i*5+j] */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) {
                sum += M[i*5+k] * A[j*5+k];
            }
            float Q_val = (i == j) ? Q_PROCESS : 0.0f;
            P_pred[i*5+j] = sum + Q_val;
        }
    }

    /*───────────────────────────── 
      5) 칼만 필터 업데이트 단계 (GPS 관측 보정)
         - 측정 모델: H = [ [1, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0] ]
         - 측정 벡터: z = [raw_gps_vx, raw_gps_vy]
         - 보정: X_updated = X_pred + K*(z - H*X_pred)
    ─────────────────────────────*/
    if(gps_update_enabled) {
        float H[10] = {
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f, 0.0f
        };
        float z_pred[2] = { X_pred[0], X_pred[1] };
        float z[2] = { raw_gps_vx, raw_gps_vy };
        float y_innov[2] = { z[0] - z_pred[0], z[1] - z_pred[1] };

        float S[4];
        S[0] = P_pred[0] + R_GPS; 
        S[1] = P_pred[1]; 
        S[2] = P_pred[5]; 
        S[3] = P_pred[6] + R_GPS;

        float S_inv[4];
        if(!Invert2x2(S, S_inv)) {
            gps_update_enabled = false;
        } else {
            float K_gain[10] = {0};  
            for (int i = 0; i < 5; i++) {
                K_gain[i*2 + 0] = P_pred[i*5 + 0] * S_inv[0] + P_pred[i*5 + 1] * S_inv[2];
                K_gain[i*2 + 1] = P_pred[i*5 + 0] * S_inv[1] + P_pred[i*5 + 1] * S_inv[3];
            }

            for (int i = 0; i < 5; i++) {
                X_pred[i] += K_gain[i*2 + 0] * y_innov[0] + K_gain[i*2 + 1] * y_innov[1];
            }

            float I_KH[25];
            memset(I_KH, 0, sizeof(I_KH));
            for (int i = 0; i < 5; i++) {
                I_KH[i*5 + i] = 1.0f;
            }
            for (int i = 0; i < 5; i++) {
                I_KH[i*5 + 0] -= K_gain[i*2 + 0];
                I_KH[i*5 + 1] -= K_gain[i*2 + 1];
            }

            float P_updated[25] = {0};
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    float sum = 0.0f;
                    for (int k = 0; k < 5; k++) {
                        sum += I_KH[i*5 + k] * P_pred[k*5 + j];
                    }
                    P_updated[i*5 + j] = sum;
                }
            }
            memcpy(kfState->P, P_updated, sizeof(P_updated));
        }
    }

    /* gps_update_enabled false이면, 예측값 그대로 사용 (P_pred를 kfState->P에 저장) */
    if(!gps_update_enabled) {
        memcpy(kfState->P, P_pred, sizeof(P_pred));
    }


    /*───────────────────────────── 
      6) 최종 상태 업데이트 및 출력 할당
    ─────────────────────────────*/
    /* 상태 벡터: X = [vx, vy, ax, ay, heading] */
    memcpy(kfState->X, X_pred, sizeof(X_pred));

    egoData->Ego_Velocity_X     = kfState->X[0];
    egoData->Ego_Velocity_Y     = kfState->X[1];
    egoData->Ego_Acceleration_X = kfState->X[2];
    egoData->Ego_Acceleration_Y = kfState->X[3];
    egoData->Ego_Heading        = kfState->X[4];
}

/*─────────────────────────────
  초기화 함수 예시: 칼만 필터 상태 초기화
─────────────────────────────*/
void InitEgoVehicleKFState(EgoVehicleKFState_t *kfState)
{
    if(!kfState) return;
    memset(kfState, 0, sizeof(*kfState));
    /* 상태 벡터 초기화: [vx, vy, ax, ay, heading] */
    kfState->X[0] = 0.0f; /* vx */
    kfState->X[1] = 0.0f; /* vy */
    kfState->X[2] = 0.0f; /* ax */
    kfState->X[3] = 0.0f; /* ay */
    kfState->X[4] = 0.0f; /* heading */

    /* 공분산 행렬 P 초기값: 단위행렬로 초기화 (또는 설계 파라미터에 따른 값) */
    for (int i = 0; i < 25; i++) {
        kfState->P[i] = 0.0f;
    }
    for (int i = 0; i < 5; i++) {
        kfState->P[i*5+i] = 100.0f;
    }

    /* 기타 내부 변수 초기화 */
    kfState->Previous_Update_Time = 0.0f;
    kfState->Prev_Accel_X = 0.0f;
    kfState->Prev_Accel_Y = 0.0f;
    kfState->Prev_Yaw_Rate = 0.0f;
    kfState->Prev_GPS_Vel_X = 0.0f;
    kfState->Prev_GPS_Vel_Y = 0.0f;
}
