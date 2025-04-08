#include <math.h>
#include <stdio.h>
#include "aeb.h"

/**
 * @brief 2.2.3.1.1 calculate_ttc_for_aeb
 * - Relative_Speed = Ego_Velocity_X - AEB_Target_Velocity_X (Ego가 더 빠를 때만 충돌 위험)
 * - TTC = Distance / Relative_Speed
 * - TTC_Brake = Ego_Velocity_X / Max_Brake_Deceleration (양수 계산)
 * - TTC_Alert = TTC_Brake + Alert_Buffer_Time
 */
void calculate_ttc_for_aeb(const AEB_Target_Data_t *pAebTargetData,
                           const Ego_Data_t        *pEgoData,
                           TTC_Data_t              *pTtcData)
{
    if(!pAebTargetData || !pEgoData || !pTtcData)
        return; /* 안전 처리 */

    /* 초기값 세팅 */
    pTtcData->TTC            = 99999.0f;  /* ∞ 로 가정 */
    pTtcData->TTC_Brake      = 0.0f;
    pTtcData->TTC_Alert      = 0.0f;
    pTtcData->Relative_Speed = 0.0f;

    /* 1) 타겟 유효성 확인 */
    if( pAebTargetData->AEB_Target_ID < 0 ||
        pAebTargetData->AEB_Target_Situation == AEB_TARGET_CUT_OUT )
    {
        /* 타겟 없거나 cut-out 상태 -> TTC=∞, 그대로 리턴 */
        return;
    }

    /* 2) 상대 속도 계산 */
    float relSpeed = pEgoData->Ego_Velocity_X - pAebTargetData->AEB_Target_Velocity_X;
    if(relSpeed <= 0.0f)
    {
        /* Ego가 더 느리거나 같은 속도면 충돌 없음 -> TTC=∞ */
        return;
    }
    pTtcData->Relative_Speed = relSpeed;

    /* 3) 거리 */
    float dist = pAebTargetData->AEB_Target_Distance;
    if(dist < 0.01f) dist = 0.01f; /* 0으로 나눗셈 방지 */

    /* 4) TTC 계산 */
    float ttc = dist / relSpeed;
    pTtcData->TTC = ttc;

    /* 5) TTC_Brake = Ego_Velocity_X / (기본 최대 감속성능) */
    if(pEgoData->Ego_Velocity_X > 0.1f)
    {
        pTtcData->TTC_Brake = pEgoData->Ego_Velocity_X / AEB_DEFAULT_MAX_DECEL; /* 9.0 m/s^2 */
    }

    /* 6) TTC_Alert = TTC_Brake + 여유시간(Alert_Buffer_Time) */
    pTtcData->TTC_Alert = pTtcData->TTC_Brake + AEB_ALERT_BUFFER_TIME;
}

/**
 * @brief 2.2.3.1.2 aeb_mode_selection
 * - Normal: 충돌 위험 없음
 * - Alert : 충돌 경고 단계
 * - Brake : 긴급 제동 단계
 */
AEB_Mode_e aeb_mode_selection(const AEB_Target_Data_t *pAebTargetData,
                              const Ego_Data_t        *pEgoData,
                              const TTC_Data_t        *pTtcData)
{
    if(!pAebTargetData || !pEgoData || !pTtcData)
        return AEB_MODE_NORMAL;

    float ttc       = pTtcData->TTC;
    float ttcBrake  = pTtcData->TTC_Brake;
    float ttcAlert  = pTtcData->TTC_Alert;

    /* AEB 작동 불가 조건: 타겟 없는 경우, 속도 너무 낮음, TTC 무효/∞ 등 */
    if(pAebTargetData->AEB_Target_ID < 0 ||
       pEgoData->Ego_Velocity_X < 0.5f  ||
       ttc <= 0.0f || ttc >= 99999.0f)
    {
        return AEB_MODE_NORMAL;
    }
    if(pAebTargetData->AEB_Target_Situation == AEB_TARGET_CUT_OUT)
    {
        return AEB_MODE_NORMAL;
    }

    /* 설계서 2.2.3.1.2: 
       - TTC > TTC_Alert => Normal
       - TTC_Brake < TTC ≤ TTC_Alert => Alert
       - 0 < TTC ≤ TTC_Brake => Brake
       - (Cut-in 상황일 때 Alert/Brake 조건도 동일하게)
    */

    if(ttc > ttcAlert)
    {
        return AEB_MODE_NORMAL;
    }
    else if(ttc > ttcBrake && ttc <= ttcAlert)
    {
        return AEB_MODE_ALERT;
    }
    else if(ttc > 0.0f && ttc <= ttcBrake)
    {
        return AEB_MODE_BRAKE;
    }

    return AEB_MODE_NORMAL;
}

/**
 * @brief 2.2.3.1.3 calculate_decel_for_aeb
 * - AEB_Mode가 Normal/Alert일 땐 0.0
 * - Brake일 땐 TTC 기반으로 선형 감속도 계산 후 -10 ~ -2 범위로 Clamping
 */
float calculate_decel_for_aeb(AEB_Mode_e aebMode,
                              const TTC_Data_t *pTtcData)
{
    if(!pTtcData) return 0.0f;

    if(aebMode == AEB_MODE_NORMAL || aebMode == AEB_MODE_ALERT)
    {
        /* 감속 없음 */
        return 0.0f;
    }
    else if(aebMode == AEB_MODE_BRAKE)
    {
        float ttc       = pTtcData->TTC;
        float ttcBrake  = pTtcData->TTC_Brake;
        if(ttcBrake < 1e-5f) ttcBrake = 1e-5f; /* 0 나눗셈 방지 */

        /* 설계서 예: Decel_AEB_X = Max_Brake_Decel * (1 - TTC / TTC_Brake) */
        /* Max_Brake_Decel은 -10 m/s^2, Min_Brake_Decel은 -2 m/s^2 */
        float ratio = 1.0f - (ttc / ttcBrake);
        float decel = AEB_MAX_BRAKE_DECEL * ratio;

        /* 범위 제한: [-10, -2] */
        if(decel > AEB_MIN_BRAKE_DECEL)
        {
            decel = AEB_MIN_BRAKE_DECEL;
        }
        if(decel < AEB_MAX_BRAKE_DECEL)
        {
            decel = AEB_MAX_BRAKE_DECEL;
        }
        return decel;
    }

    /* 기본값 */
    return 0.0f;
}
