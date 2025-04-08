#ifndef ARBITRATION_H
#define ARBITRATION_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float throttle; /* 0.0 ~ 1.0 */
    float brake;    /* 0.0 ~ 1.0 */
    float steer;    /* -1.0 ~ 1.0 */
} VehicleControl_t;

/**
 * @brief Arbitration_ComputeControl
 * @param accelAccX   : ACC 가속도 (-10~10)
 * @param decelAebX   : AEB 감속도 (-10~0)
 * @param steerLfa    : LFA 조향각(단위 deg, -540~540)
 * @param aebMode     : AEB 모드
 * @param pOutControl : 최종 VehicleControl
 */
void Arbitration_ComputeControl(float accelAccX,
                                float decelAebX,
                                float steerLfa,
                                AEB_Mode_e aebMode,
                                VehicleControl_t *pOutControl);

#ifdef __cplusplus
}
#endif

#endif /* ARBITRATION_H */
