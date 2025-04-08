#ifndef EGO_VEHICLE_ESTIMATION_H
#define EGO_VEHICLE_ESTIMATION_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * 칼만 필터 상태 구조체 (설계서 2.2.2 '내부 변수'와 매칭)
 *  - Last_GPS_Velocity_X/Y
 *  - Last_GPS_Timestamp
 *  - Previous_Update_Time
 *  - Prev_Accel_X, Prev_Accel_Y, Prev_Yaw_Rate
 *  - Prev_GPS_Vel_X, Prev_GPS_Vel_Y
 *  - X[], P[] : 칼만 필터 상태벡터 & 공분산행렬
 */
typedef struct {
    float Last_GPS_Velocity_X;
    float Last_GPS_Velocity_Y;
    float Last_GPS_Timestamp;

    float Previous_Update_Time;

    float Prev_Accel_X;
    float Prev_Accel_Y;
    float Prev_Yaw_Rate;
    float Prev_GPS_Vel_X;
    float Prev_GPS_Vel_Y;

    /* 칼만 필터 상태 [vx, vy, ax, ay, heading] 등 */
    float X[5];
    float P[25]; /* 5x5 공분산 */
} EgoEstimationKFState_t;

/*=== 노이즈 임계값, GPS 유효성 시간 ===*/
/* 설계서상 ±3.0, ±30, ±10, 50ms 등 */
#define MAX_SENSOR_NOISE_ACCEL    3.0f   /* IMU 가속도 스파이크 임계값 */
#define MAX_SENSOR_NOISE_YAWRATE  30.0f  /* IMU Yaw Rate 스파이크 임계값 */
#define MAX_SENSOR_NOISE_GPSVEL   10.0f  /* GPS 속도 스파이크 임계값 */
#define GPS_VALID_TIME_THRESH     50.0f  /* [ms], GPS 유효성 판단 */

/*=== 함수 프로토타입 ===*/
/* 설계서 2.2.2에서 "Ego Vehicle Estimation" 모듈 - Init/Update 예시 */

/* 초기화: 칼만 필터 내부 상태, 이전값 등 */
void EgoVehicleEstimation_Init(EgoEstimationKFState_t *pState);

/* 메인 함수: IMU+GPS 융합하여 EgoData 추정 */
void EgoVehicleEstimation(
    const TimeData_t        *timeData,
    const GPSData_t         *gpsData,
    const IMUData_t         *imuData,
    EgoData_t               *pEgoData,
    EgoEstimationKFState_t  *pState
);

#ifdef __cplusplus
}
#endif

#endif /* EGO_ESTIMATION_H */
