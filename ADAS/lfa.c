#include <math.h>
#include <stdio.h>
#include "lfa.h"

/* PID 적분, 이전오차 */
static float g_pidIntegral=0.0f;
static float g_pidPrevErr=0.0f;

LFA_Mode_e LFA_ModeSelection(const EgoData_t *pEgoData)
{
    if(!pEgoData) return LFA_MODE_LOW_SPEED;
    /* 시속 60km/h => 16.67 m/s */
    if(pEgoData->Ego_Velocity_X < LFA_LOW_SPEED_THRESHOLD){
        return LFA_MODE_LOW_SPEED;
    } 
    return LFA_MODE_HIGH_SPEED;
}

float LFA_CalcSteer_LowSpeedPID(const LaneSelectOutput_t *pLsData,
                                float deltaTime)
{
    if(!pLsData) return 0.0f;
    /* 차선 중심오차 + heading 오차 */
    float offsetErr= pLsData->LS_Lane_Offset;
    float hdgErr   = pLsData->LS_Heading_Error;
    /* 단순 합산 */
    float K_offset=1.0f, K_heading=1.0f;
    float error=(K_offset*offsetErr)+(K_heading*hdgErr);

    /* PID gains */
    float Kp=0.1f,Ki=0.01f,Kd=0.005f;
    g_pidIntegral += error*deltaTime;
    float dErr=(error-g_pidPrevErr)/(deltaTime+1e-5f);
    g_pidPrevErr=error;

    float steer=(Kp*error)+(Ki*g_pidIntegral)+(Kd*dErr);
    /* clamp */
    if(steer> LFA_MAX_STEERING_ANGLE) steer=LFA_MAX_STEERING_ANGLE;
    if(steer<-LFA_MAX_STEERING_ANGLE) steer=-LFA_MAX_STEERING_ANGLE;
    return steer;
}

float LFA_CalcSteer_HighSpeedStanley(const EgoData_t *pEgoData,
                                     const LaneSelectOutput_t *pLsData)
{
    if(!pEgoData||!pLsData) return 0.0f;
    float vx= pEgoData->Ego_Velocity_X;
    if(vx<0.1f) vx=0.1f; /* 분모 보호 */

    float offset= pLsData->LS_Lane_Offset;
    float hdgErr= pLsData->LS_Heading_Error;
    /* deg->rad? 간단히 deg 연산 */
    float stGain=1.0f;

    float angleOffset= atanf((stGain*offset)/vx)*(180.0f/(float)M_PI);
    float steer= hdgErr + angleOffset;
    if(steer> LFA_MAX_STEERING_ANGLE) steer=LFA_MAX_STEERING_ANGLE;
    if(steer<-LFA_MAX_STEERING_ANGLE) steer=-LFA_MAX_STEERING_ANGLE;

    return steer;
}

float LFA_OutputSelection(LFA_Mode_e lfaMode,
                          float steerPid,
                          float steerStanley,
                          const LaneSelectOutput_t *pLsData,
                          const EgoData_t *pEgoData)
{
    float steering=0.0f;
    if(lfaMode==LFA_MODE_LOW_SPEED){
        steering= steerPid;
    } else {
        steering= steerStanley;
    }

    /* 차선 변경중 => 감쇠 */
    if(pLsData->LS_Is_Changing_Lane){
        steering*=0.2f; 
    }

    /* 차선 이탈 => 복귀 강화 */
    if(!pLsData->LS_Is_Within_Lane){
        steering*=1.5f;
    }

    /* 곡선 => 민감도 증가 */
    float curveGain=1.0f;
    if(pLsData->LS_Is_Curved_Lane){
        curveGain=1.2f;
        if(pEgoData){
            float yawThreshold=30.0f;
            float steerThreshold=200.0f;
            if(fabsf(pEgoData->Ego_Yaw_Rate)>yawThreshold ||
               fabsf(pEgoData->Ego_Heading)>steerThreshold){
                curveGain=0.8f; 
            }
        }
    }
    steering*=curveGain;

    /* clamp */
    if(steering> LFA_MAX_STEERING_ANGLE) steering=LFA_MAX_STEERING_ANGLE;
    if(steering<-LFA_MAX_STEERING_ANGLE)steering=-LFA_MAX_STEERING_ANGLE;

    return steering;
}
