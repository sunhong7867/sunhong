#ifndef AEB_H
#define AEB_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AEB: calculate TTC
 * @param pEgoData
 * @param pAebTarget
 * @param pTTC
 * @param pTTC_Brake
 * @param pTTC_Alert
 * @param pRelativeSpeed
 */
void AEB_CalcTTC(const EgoData_t *pEgoData,
                 const AEB_Target_t *pAebTarget,
                 float *pTTC,
                 float *pTTC_Brake,
                 float *pTTC_Alert,
                 float *pRelativeSpeed);

/**
 * @brief AEB 모드 결정
 * @param ttc, ttcBrake, ttcAlert
 * @param pAebTarget
 * @param pEgoData
 * @return AEB_Mode_e
 */
AEB_Mode_e AEB_ModeSelection(float ttc, float ttcBrake, float ttcAlert,
                             const AEB_Target_t *pAebTarget,
                             const EgoData_t *pEgoData);

/**
 * @brief AEB 감속도 계산
 * @param aebMode
 * @param ttc
 * @param ttcBrake
 * @return float decel (-10 ~ 0)
 */
float AEB_CalcDecel(AEB_Mode_e aebMode,
                    float ttc, 
                    float ttcBrake);

#ifdef __cplusplus
}
#endif

#endif /* AEB_H */
