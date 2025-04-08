#ifndef ACC_H
#define ACC_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ACC 모드 결정
 * @param pAccTarget : 최종 선정된 ACC 타겟
 * @param pEgoData   : Ego 차량 
 * @param pLsData    : 차선 곡률 
 * @return ACCMode_e
 */
ACCMode_e ACC_ModeSelection(const ACC_Target_t *pAccTarget,
                            const EgoData_t *pEgoData,
                            const LaneSelectOutput_t *pLsData);

/**
 * @brief ACC 거리 모드(Stop 포함) 가속도 계산
 * @param pAccTarget
 * @param pEgoData
 * @param deltaTime
 * @return float: 종방향 가속도 (-10~+10)
 */
float ACC_CalcAccel_Distance(const ACC_Target_t *pAccTarget,
                             const EgoData_t *pEgoData,
                             float deltaTime);

/**
 * @brief ACC 속도 모드 가속도 계산
 * @param pEgoData
 * @param pLsData (곡선 -> 감속 보정)
 * @param deltaTime
 * @return float: 종방향 가속도
 */
float ACC_CalcAccel_Speed(const EgoData_t *pEgoData,
                          const LaneSelectOutput_t *pLsData,
                          float deltaTime);

/**
 * @brief ACC 최종 출력 선택
 * @param accMode : ACCMode_e
 * @param accelDist : 거리모드 계산결과
 * @param accelSpeed: 속도모드 계산결과
 * @return float: 최종 ACC 종방향 가속도
 */
float ACC_OutputSelection(ACCMode_e accMode,
                          float accelDist,
                          float accelSpeed);

#ifdef __cplusplus
}
#endif

#endif /* ACC_H */
