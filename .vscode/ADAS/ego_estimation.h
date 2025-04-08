#ifndef EGO_ESTIMATION_H
#define EGO_ESTIMATION_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 칼만 필터 상태 예시 구조체 */
typedef struct {
    float Last_GPS_VelX, Last_GPS_VelY;
    float Last_GPS_Timestamp;
    float Prev_Update_Time;

    /* IMU / GPS 이전값 (스파이크 제거용) */
    float Prev_Accel_X;
    float Prev_Accel_Y;
    float Prev_YawRate;
    float Prev_GPS_VelX;
    float Prev_GPS_VelY;

    /* 칼만 필터 상태 [vx, vy, ax, ay, heading] 등 */
    float state[5];
    float P[25]; /* 5x5 공분산 */
} EgoEstimationKFState_t;

/* 노이즈 임계값, GPS 유효성 등 */
#define MAX_SENSOR_NOISE_ACCEL  3.0f
#define MAX_SENSOR_NOISE_YAWRATE 30.0f
#define MAX_SENSOR_NOISE_GPSVEL 10.0f
#define GPS_VALID_TIME_THRESH   50.0f  /* ms */

/* 함수 프로토타입 */
void EgoEstimation_Init(EgoEstimationKFState_t *pState);

void EgoEstimation_Update(const TimeData_t *timeData,
                          const GPSData_t *gpsData,
                          const IMUData_t *imuData,
                          EgoData_t       *pEgoData,
                          EgoEstimationKFState_t *pState);

#ifdef __cplusplus
}
#endif

#endif /* EGO_ESTIMATION_H */
