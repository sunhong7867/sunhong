#include <math.h>
#include <stdio.h>
#include "acc.h"

/* PID 적분/이전오차 (Distance) */
static float g_distIntegral=0.0f;
static float g_distPrevErr=0.0f;
/* PID 적분/이전오차 (Speed) */
static float g_speedIntegral=0.0f;
static float g_speedPrevErr=0.0f;

ACCMode_e ACC_ModeSelection(const ACC_Target_t *pAccTarget,
                            const EgoData_t *pEgoData,
                            const LaneSelectOutput_t *pLsData)
{
    /* 설계서 2.2.5 => (1) ACC 모드 판단 */
    if(!pAccTarget || !pEgoData) return ACC_MODE_SPEED; /* fallback */

    /* 만약 타겟ID==-1 => 타겟 없음 => speed */
    if(pAccTarget->ACC_Target_ID<0) {
        return ACC_MODE_SPEED;
    }

    float dist = pAccTarget->ACC_Target_Distance;
    if(dist>55.0f) {
        return ACC_MODE_SPEED;
    }
    else if(dist<45.0f) {
        return ACC_MODE_DISTANCE;
    }
    else {
        /* 45~55 => 이전모드 유지 => 여기선 그냥 distance or speed 중 임시 */
        /* 혹은 "if TargetStatus==Stopped => STOP" */
        if(pAccTarget->ACC_Target_Status==OBJSTAT_STOPPED && (pEgoData->Ego_Velocity_X<0.5f)){
            return ACC_MODE_STOP;
        }
        /* 여기서는 간단히 SPEED */
        return ACC_MODE_SPEED;
    }
}

float ACC_CalcAccel_Distance(const ACC_Target_t *pAccTarget,
                             const EgoData_t *pEgoData,
                             float deltaTime)
{
    if(!pAccTarget||!pEgoData) return 0.0f;

    /* 기준거리=40, PID */
    float targetDist=40.0f;
    float distErr= targetDist - pAccTarget->ACC_Target_Distance;

    /* PID gains */
    float Kp=0.4f, Ki=0.05f, Kd=0.1f;

    g_distIntegral += distErr*deltaTime;
    float dErr=(distErr-g_distPrevErr)/(deltaTime+1e-5f);
    g_distPrevErr=distErr;

    float accelDist = (Kp*distErr)+(Ki*g_distIntegral)+(Kd*dErr);

    /* STOP 모드도 여기서 처리 (정지) */
    if(pAccTarget->ACC_Target_Status==OBJSTAT_STOPPED && 
       pEgoData->Ego_Velocity_X<0.5f){
        /* -3.0 => 강제정지 */
        accelDist=-3.0f;
    }
    return accelDist;
}

float ACC_CalcAccel_Speed(const EgoData_t *pEgoData,
                          const LaneSelectOutput_t *pLsData,
                          float deltaTime)
{
    if(!pEgoData||!pLsData) return 0.0f;

    float baseTargetSpeed=22.22f; /* 80km/h => 22.22 m/s */
    /* 곡선시 15m/s로 제한 */
    if(pLsData->LS_Is_Curved_Lane){
        if(baseTargetSpeed>15.0f) baseTargetSpeed=15.0f;
    }

    float speedErr= baseTargetSpeed - pEgoData->Ego_Velocity_X;
    float Kp=0.5f, Ki=0.1f, Kd=0.05f;

    g_speedIntegral += speedErr*deltaTime;
    float dErr=(speedErr-g_speedPrevErr)/(deltaTime+1e-5f);
    g_speedPrevErr=speedErr;

    float accelSpeed=(Kp*speedErr)+(Ki*g_speedIntegral)+(Kd*dErr);
    return accelSpeed;
}

float ACC_OutputSelection(ACCMode_e accMode,
                          float accelDist,
                          float accelSpeed)
{
    if(accMode==ACC_MODE_SPEED){
        return accelSpeed;
    }
    else if(accMode==ACC_MODE_DISTANCE){
        return accelDist;
    }
    else if(accMode==ACC_MODE_STOP){
        /* 정지 => 0.0 or -값 */
        return 0.0f; 
    }
    return 0.0f;
}
