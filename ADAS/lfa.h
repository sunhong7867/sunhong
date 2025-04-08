#ifndef LFA_H
#define LFA_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 저속/고속 모드 판단 */
LFA_Mode_e LFA_ModeSelection(const EgoData_t *pEgoData);

/* 저속 PID */
float LFA_CalcSteer_LowSpeedPID(const LaneSelectOutput_t *pLsData,
                                float deltaTime);

/* 고속 Stanley */
float LFA_CalcSteer_HighSpeedStanley(const EgoData_t *pEgoData,
                                     const LaneSelectOutput_t *pLsData);

/* 최종 조향 결정 */
float LFA_OutputSelection(LFA_Mode_e lfaMode,
                          float steerPid,
                          float steerStanley,
                          const LaneSelectOutput_t *pLsData,
                          const EgoData_t *pEgoData);

#ifdef __cplusplus
}
#endif

#endif /* LFA_H */
