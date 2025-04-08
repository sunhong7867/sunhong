#ifndef ARBITRATION_H
#define ARBITRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "aeb.h"  /* AEB_Mode_e 같은 열거형이 정의돼있다고 가정 */
                 /* or 필요한 enum/struct를 여기서 재정의 */

/**
 * @brief 최종 VehicleControl 신호 구조
 *        (Carla vehicle_control.throttle, brake, steer 에 대응)
 */
typedef struct
{
    float throttle; /* (0.0 ~ 1.0) */
    float brake;    /* (0.0 ~ 1.0) */
    float steer;    /* (-1.0 ~ 1.0) */
} VehicleControl_t;

/**
 * @brief Arbitration (2.2.8.1.1)
 *        ACC, AEB, LFA 출력값을 받아서 최종 제어 신호로 변환
 *
 * @param accelAccX  : (입력) ACC 종방향 가속도 (-10~+10) [m/s^2]
 * @param decelAebX  : (입력) AEB 종방향 감속도 (-10~0) [m/s^2]
 * @param steerLfa   : (입력) LFA 조향각(°), -540~540
 * @param aebMode    : (입력) AEB 모드 (Normal, Alert, Brake)
 * @param pOutControl: (출력) 최종 throttle, brake, steer
 */
void Arbitration(float accelAccX,
                 float decelAebX,
                 float steerLfa,
                 AEB_Mode_e aebMode,
                 VehicleControl_t *pOutControl);

#ifdef __cplusplus
}
#endif

#endif /* ARBITRATION_H */
