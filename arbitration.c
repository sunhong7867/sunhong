#include <math.h>
#include <stdio.h>
#include "arbitration.h"

static const float MAX_THROTTLE_ACCEL=10.0f; 
static const float MAX_BRAKE_DECEL   =-10.0f; 
static const float MAX_STEER_ANGLE   =540.0f;

void Arbitration_ComputeControl(float accelAccX,
                                float decelAebX,
                                float steerLfa,
                                AEB_Mode_e aebMode,
                                VehicleControl_t *pOutControl)
{
    if(!pOutControl) return;

    float selectedAccel=0.0f;
    /* 1) 종방향 가속도 선택 */
    if(aebMode==AEB_MODE_BRAKE){
        selectedAccel= decelAebX; /* -10~0 */
    }
    else if(aebMode==AEB_MODE_NORMAL||aebMode==AEB_MODE_ALERT){
        selectedAccel= accelAccX; /* -10~+10 */
    }
    else {
        selectedAccel= 0.0f;
    }

    /* 2) throttle / brake 정규화 */
    float throttle=0.0f;
    float brake=0.0f;

    if(selectedAccel>0.0f){
        float ratio= selectedAccel/ MAX_THROTTLE_ACCEL; 
        if(ratio>1.0f) ratio=1.0f;
        if(ratio<0.0f) ratio=0.0f;
        throttle= ratio;
        brake   = 0.0f;
    }
    else if(selectedAccel<0.0f){
        float ratio= fabsf(selectedAccel/ MAX_BRAKE_DECEL); 
        if(ratio>1.0f) ratio=1.0f;
        brake= ratio;
        throttle=0.0f;
    }
    else {
        throttle=0.0f;
        brake=0.0f;
    }

    /* 3) 조향각 */
    float steerRatio= steerLfa/ MAX_STEER_ANGLE;
    if(steerRatio> 1.0f) steerRatio=1.0f;
    if(steerRatio<-1.0f) steerRatio=-1.0f;

    /* 4) 최종 output */
    pOutControl->throttle= throttle; /* 0~1 */
    pOutControl->brake   = brake;    /* 0~1 */
    pOutControl->steer   = steerRatio; /* -1~1 */
}
