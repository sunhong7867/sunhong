#include <math.h>
#include "arbitration.h"

/* 상수 정의 */
static const float MAX_THROTTLE_ACCEL = 10.0f;   /* +10 m/s^2 일 때 throttle=1.0 */
static const float MAX_BRAKE_DECEL    =-10.0f;   /* -10 m/s^2 일 때 brake=1.0 */
static const float MAX_STEER_ANGLE    = 540.0f;  /* ±540도 -> steer ±1.0 */

/* ----------------------------------------------------------------------------
 * (2.2.8.1.1) Arbitration
 *   - AEB가 Brake 모드면 AEB 감속 우선
 *   - 그 외엔 ACC 가속도 사용
 *   - steer는 LFA
 *   - throttle/brake/steer 정규화
 * ---------------------------------------------------------------------------*/
void Arbitration(float accelAccX,
                 float decelAebX,
                 float steerLfa,
                 AEB_Mode_e aebMode,
                 VehicleControl_t *pOutControl)
{
    if(!pOutControl)
        return;

    float selectedAccel = 0.0f;

    /* 1) 종방향 가속도 선택 */
    if(aebMode == AEB_MODE_BRAKE)
    {
        /* 충돌 임박 => AEB의 감속이 최우선 */
        selectedAccel = decelAebX;  /* (-10 ~ 0) */
    }
    else
    {
        /* AEB Normal/Alert 일 때, ACC 출력을 사용 */
        selectedAccel = accelAccX; /* (-10 ~ +10) */
    }

    /* 2) throttle / brake 계산 */
    float throttleCmd = 0.0f;
    float brakeCmd    = 0.0f;

    if(selectedAccel > 0.0f)
    {
        /* 가속 => throttle = accel / 10.0 (최대) */
        float ratio = selectedAccel / MAX_THROTTLE_ACCEL;
        if(ratio > 1.0f) ratio = 1.0f;
        if(ratio < 0.0f) ratio = 0.0f;
        throttleCmd = ratio; // 0~1
        brakeCmd    = 0.0f;
    }
    else if(selectedAccel < 0.0f)
    {
        /* 감속 => brake = |accel| / 10.0 (최대) */
        float ratio = fabsf(selectedAccel / MAX_BRAKE_DECEL); // note: MAX_BRAKE_DECEL= -10
        if(ratio > 1.0f) ratio = 1.0f;
        if(ratio < 0.0f) ratio = 0.0f;
        brakeCmd    = ratio; // 0~1
        throttleCmd = 0.0f;
    }
    else
    {
        /* accel=0 => 유지 => throttle=0, brake=0 */
        throttleCmd = 0.0f;
        brakeCmd    = 0.0f;
    }

    /* 3) 조향각 정규화 (-540~540 => -1.0~1.0) */
    float steerRatio = steerLfa / MAX_STEER_ANGLE;
    if(steerRatio >  1.0f) steerRatio =  1.0f;
    if(steerRatio < -1.0f) steerRatio = -1.0f;

    /* 4) 최종 출력 */
    pOutControl->throttle = throttleCmd; // (0.0~1.0)
    pOutControl->brake    = brakeCmd;    // (0.0~1.0)
    pOutControl->steer    = steerRatio;  // (-1.0~1.0)
}
