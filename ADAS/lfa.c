#include <math.h>
#include <stdio.h>
#include "lfa.h"

/* 예시: 속도 기준 (시속 60 km/h = 16.67 m/s) */
#define LFA_SPEED_THRESHOLD (16.67f)

/* PID 제어를 위한 내부 변수 (저속) */
static float s_pidIntegral  = 0.0f;
static float s_pidPrevError = 0.0f;

/* 시스템에서 정의한 최대 조향각(±540°) */
static const float LFA_MAX_STEERING_ANGLE = 540.0f;

/* ----------------------------------------------------------------------------
 * (2.2.4.1.1) lfa_mode_selection
 *   - 60 km/h(16.67m/s) 기준으로 LOW_SPEED / HIGH_SPEED 모드 분기
 * ---------------------------------------------------------------------------*/
LFA_Mode_e lfa_mode_selection(const Ego_Data_t *pEgoData)
{
    if(!pEgoData)
    {
        return LFA_MODE_LOW_SPEED; // fallback
    }

    if(pEgoData->Ego_Velocity_X < LFA_SPEED_THRESHOLD)
    {
        return LFA_MODE_LOW_SPEED;
    }
    else
    {
        return LFA_MODE_HIGH_SPEED;
    }
}

/* ----------------------------------------------------------------------------
 * (2.2.4.1.2) calculate_steer_in_low_speed_pid
 *  - PID 기반으로 Heading Error + Lane Offset 반영
 * ---------------------------------------------------------------------------*/
float calculate_steer_in_low_speed_pid(const Lane_Data_LS_t *pLaneData,
                                       float deltaTime)
{
    if(!pLaneData || deltaTime <= 0.0f)
        return 0.0f;

    /* 오차 계산: 가중합 (K_offset * LaneOffset + K_heading * HeadingError) */
    float K_offset  = 1.0f;  // 비중 예시
    float K_heading = 1.0f;
    float offsetErr = pLaneData->LS_Lane_Offset;    /* (-2.0~2.0) m */
    float hdgErr    = pLaneData->LS_Heading_Error;  /* (-180~180)° */

    float error = (K_offset * offsetErr) + (K_heading * hdgErr);

    /* PID 게인 (설계서 예시) */
    float Kp = 0.1f;
    float Ki = 0.01f;
    float Kd = 0.005f;

    /* 누적오차(적분항) */
    s_pidIntegral += (error * deltaTime);

    /* 미분항 */
    float dErr = (error - s_pidPrevError) / (deltaTime + 1e-6f);
    s_pidPrevError = error;

    /* PID 계산 */
    float steeringAnglePID = (Kp * error) + (Ki * s_pidIntegral) + (Kd * dErr);

    /* 제한 (±540도) */
    if(steeringAnglePID >  LFA_MAX_STEERING_ANGLE)  steeringAnglePID =  LFA_MAX_STEERING_ANGLE;
    if(steeringAnglePID < -LFA_MAX_STEERING_ANGLE)  steeringAnglePID = -LFA_MAX_STEERING_ANGLE;

    return steeringAnglePID;
}

/* ----------------------------------------------------------------------------
 * (2.2.4.1.3) calculate_steer_in_high_speed_stanley
 *  - Stanley 제어 공식: steer = headingError + atan2(k * cte, velocity)
 *    여기서는 cte를 LS_Lane_Offset이라 간주
 * ---------------------------------------------------------------------------*/
float calculate_steer_in_high_speed_stanley(const Ego_Data_t     *pEgoData,
                                            const Lane_Data_LS_t *pLaneData)
{
    if(!pEgoData || !pLaneData)
        return 0.0f;

    float vx = pEgoData->Ego_Velocity_X;
    if(vx < 0.1f) vx = 0.1f; // 분모 보호

    float headingErr = pLaneData->LS_Heading_Error;   // (°)
    float cte        = pLaneData->LS_Lane_Offset;     // cross-track error (m)

    /* Stanley 제어에서 사용할 Gain (예시 1.0) */
    float stanleyGain = 1.0f;

    /* atan2(...) 결과는 rad. -> deg 변환할지 여부는 시스템 설계에 따라 */
    float steerOffsetRad = atanf((stanleyGain * cte) / vx);
    float steerOffsetDeg = (steerOffsetRad * 180.0f) / 3.1415926535f;

    /* 최종 조향각(°) = headingErr + offsetDeg */
    float steeringAngleStanley = headingErr + steerOffsetDeg;

    /* clamp */
    if(steeringAngleStanley >  LFA_MAX_STEERING_ANGLE)  steeringAngleStanley =  LFA_MAX_STEERING_ANGLE;
    if(steeringAngleStanley < -LFA_MAX_STEERING_ANGLE)  steeringAngleStanley = -LFA_MAX_STEERING_ANGLE;

    return steeringAngleStanley;
}

/* ----------------------------------------------------------------------------
 * (2.2.4.1.4) lfa_output_selection
 *  - 모드(LOW/HIGH) 선택 → PID or Stanley
 *  - 차선 변경/이탈, 곡선 도로 여부, YawRate/SteeringAngle 등에 따라
 *    조향각 감쇠 또는 증폭
 * ---------------------------------------------------------------------------*/
float lfa_output_selection(LFA_Mode_e lfaMode,
                           float steeringAnglePID,
                           float steeringAngleStanley,
                           const Lane_Data_LS_t *pLaneData,
                           const Ego_Data_t     *pEgoData)
{
    if(!pLaneData || !pEgoData)
    {
        return 0.0f;
    }

    float steerOut = 0.0f;
    if(lfaMode == LFA_MODE_LOW_SPEED)
    {
        steerOut = steeringAnglePID;
    }
    else
    {
        steerOut = steeringAngleStanley;
    }

    /* 1) 차선 변경 중이면 자동조향 억제(감쇠) */
    if(pLaneData->LS_Is_Changing_Lane)
    {
        // 예시: 0.2 배로 줄임
        steerOut *= 0.2f;
    }

    /* 2) 차선 이탈시 복귀 강화 (증폭) */
    if(!pLaneData->LS_Is_Within_Lane)
    {
        // 예시: 1.5 배로 증가
        steerOut *= 1.5f;
    }

    /* 3) 곡선 도로 → 민감도 증가 */
    float curveGain = 1.0f;
    if(pLaneData->LS_Is_Curved_Lane)
    {
        curveGain = 1.2f;
        /* 추가적으로 YawRate, SteeringAngle 한계 시 감쇠 예시 */
        float yawRateThresh   = 30.0f;   // deg/s
        float steeringThresh  = 200.0f;  // deg
        if( (pEgoData->Ego_Yaw_Rate > yawRateThresh) ||
            (fabsf(pEgoData->Ego_Steering_Angle) > steeringThresh) )
        {
            // 만약 이미 급조향 중이면 오히려 증폭 축소
            curveGain = 0.8f;
        }
    }
    steerOut *= curveGain;

    /* clamp */
    if(steerOut >  LFA_MAX_STEERING_ANGLE) steerOut =  LFA_MAX_STEERING_ANGLE;
    if(steerOut < -LFA_MAX_STEERING_ANGLE) steerOut = -LFA_MAX_STEERING_ANGLE;

    return steerOut;
}
