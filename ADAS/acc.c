#include <math.h>
#include <stdio.h>
#include "acc.h"

/* Distance PID 적분, 과거오차 저장 */
static float s_distIntegral  = 0.0f;
static float s_distPrevError = 0.0f;

/* Speed PID 적분, 과거오차 저장 */
static float s_speedIntegral  = 0.0f;
static float s_speedPrevError = 0.0f;

/* 이전 시간 저장(예: 거리 PID에서 Delta Time 계산용) */
static float s_prevTimeDistance = 0.0f;

/**
 * @brief 2.2.4.1.1 ACC 모드 결정
 */
ACC_Mode_e acc_mode_selection(
    const ACC_Target_Data_t *pAccTargetData,
    const Ego_Data_t        *pEgoData,
    const Lane_Data_t       *pLaneData
)
{
    /* 유효성 검사 (NULL 포인터 대비) */
    if((pAccTargetData == NULL) || (pEgoData == NULL) || (pLaneData == NULL))
    {
        return ACC_MODE_SPEED; /* fallback */
    }

    /* 1) 타겟 유효성 판단: ACC_Target_ID < 0 이면 타겟 없음 */
    if(pAccTargetData->ACC_Target_ID < 0)
    {
        return ACC_MODE_SPEED;
    }

    /* 2) 거리 기반 모드 결정 */
    float dist = pAccTargetData->ACC_Target_Distance;
    if(dist > 55.0f)
    {
        /* 전방 차량이 멀거나 없음 -> Speed 모드 */
        return ACC_MODE_SPEED;
    }
    else if(dist < 45.0f)
    {
        /* 전방 차량이 가까움 -> Distance 모드 */
        return ACC_MODE_DISTANCE;
    }
    else
    {
        /* 45m <= dist <= 55m -> 이전 모드 유지 or 추가 조건 */
        /* 설계서 예시:
           - 만약 타겟 상태가 Stopped이고, Ego도 거의 정지면 Stop 모드
           - 그 외엔 Speed */
        if((pAccTargetData->ACC_Target_Status == ACC_TARGET_STOPPED) &&
           (pEgoData->Ego_Velocity_X < 0.5f))
        {
            return ACC_MODE_STOP;
        }

        return ACC_MODE_SPEED; /* 기본값 */
    }
}

/**
 * @brief 2.2.4.1.2 거리 PID 계산
 */
float calculate_accel_for_distance_pid(
    ACC_Mode_e               accMode,
    const ACC_Target_Data_t *pAccTargetData,
    const Ego_Data_t        *pEgoData,
    float                    current_time
)
{
    /* 간단 유효성 체크 */
    if((pAccTargetData == NULL) || (pEgoData == NULL))
    {
        return 0.0f;
    }

    /* Distance 모드나 Stop 모드일 때만 거리 PID 유효 */
    if((accMode != ACC_MODE_DISTANCE) && (accMode != ACC_MODE_STOP))
    {
        return 0.0f;
    }

    /* Delta Time 계산 (예: current_time ms단위 가정) */
    float deltaTime = current_time - s_prevTimeDistance;
    if(deltaTime < 0.0f)  deltaTime = 0.01f; /* fallback */
    s_prevTimeDistance = current_time;

    /* 기준거리 = 40m (설계서에서) */
    float targetDist = 40.0f;
    float distErr    = targetDist - pAccTargetData->ACC_Target_Distance;

    /* PID Gains */
    float Kp = 0.4f, Ki = 0.05f, Kd = 0.1f;

    s_distIntegral     += distErr * deltaTime;
    float dErr          = (distErr - s_distPrevError) / (deltaTime + 1e-5f);
    s_distPrevError     = distErr;

    float accelDist = (Kp * distErr) + (Ki * s_distIntegral) + (Kd * dErr);

    /* Stop 모드의 경우 (정지 유지) / Stopped 타겟과 ego도 거의 0이면 강제 제동 */
    if((pAccTargetData->ACC_Target_Status == ACC_TARGET_STOPPED) &&
       (pEgoData->Ego_Velocity_X < 0.5f))
    {
        /* -3.0 => 강제정지 */
        accelDist = -3.0f;
    }

    return accelDist;
}

/**
 * @brief 2.2.4.1.3 속도 PID 계산
 */
float calculate_accel_for_speed_pid(
    const Ego_Data_t  *pEgoData,
    const Lane_Data_t *pLaneData,
    float              delta_time
)
{
    if((pEgoData == NULL) || (pLaneData == NULL) || (delta_time <= 0.0f))
    {
        return 0.0f;
    }

    /* 기본 목표 속도: 80 km/h = 22.22 m/s */
    float baseTargetSpeed = 22.22f;

    /* 곡선 차선이면 속도 제한: 15 m/s */
    if(pLaneData->LS_Is_Curved_Lane)
    {
        if(baseTargetSpeed > 15.0f)
        {
            baseTargetSpeed = 15.0f;
        }
    }

    /* 오차 = 목표속도 - 현재속도 */
    float speedErr = baseTargetSpeed - pEgoData->Ego_Velocity_X;

    /* PID 게인 */
    float Kp = 0.5f;
    float Ki = 0.1f;
    float Kd = 0.05f;

    s_speedIntegral      += speedErr * delta_time;
    float dErr            = (speedErr - s_speedPrevError) / (delta_time + 1e-5f);
    s_speedPrevError      = speedErr;

    float accelSpeed = (Kp * speedErr) + (Ki * s_speedIntegral) + (Kd * dErr);

    return accelSpeed;
}

/**
 * @brief 2.2.4.1.4 최종 ACC 가속도 선택
 * - Speed 모드 => Accel_Speed_X
 * - Distance 모드 => Accel_Distance_X
 * - Stop 모드 => 0.0
 */
float acc_output_selection(
    ACC_Mode_e accMode,
    float      Accel_Distance_X,
    float      Accel_Speed_X
)
{
    if(accMode == ACC_MODE_SPEED)
    {
        return Accel_Speed_X;
    }
    else if(accMode == ACC_MODE_DISTANCE)
    {
        return Accel_Distance_X;
    }
    else if(accMode == ACC_MODE_STOP)
    {
        /* 정지 유지 */
        return 0.0f;
    }

    return 0.0f;
}
