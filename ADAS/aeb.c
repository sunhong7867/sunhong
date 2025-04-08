#include <math.h>
#include <stdio.h>
#include "aeb.h"

void AEB_CalcTTC(const EgoData_t *pEgoData,
                 const AEB_Target_t *pAebTarget,
                 float *pTTC,
                 float *pTTC_Brake,
                 float *pTTC_Alert,
                 float *pRelativeSpeed)
{
    if(!pEgoData||!pAebTarget||!pTTC||!pTTC_Brake||!pTTC_Alert||!pRelativeSpeed) return;
    *pTTC=99999.0f; 
    *pTTC_Brake=0.0f;
    *pTTC_Alert=0.0f;
    *pRelativeSpeed=0.0f;

    if(pAebTarget->AEB_Target_ID<0){
        return; /* 타겟없음 => TTC=∞ */
    }

    /* relative speed */
    float relSp = pEgoData->Ego_Velocity_X - pAebTarget->AEB_Target_Vel_X;
    if(relSp<=0.0f) {
        /* ego가 같거나 느리면 충돌X => TTC=∞ */
        return;
    }
    *pRelativeSpeed=relSp;
    /* distance */
    float dist = pAebTarget->AEB_Target_Distance; 
    if(dist<=0.01f) dist=0.01f;

    float ttc = dist/relSp; 
    *pTTC= ttc;

    /* TTC_Brake = egoVel / maxDecel(양수) */
    float maxDecel= AEB_DEFAULT_MAX_DECEL; /* ex. 9.0 m/s^2 */
    float tBrake=0.0f;
    if(pEgoData->Ego_Velocity_X>0.1f){
        tBrake= pEgoData->Ego_Velocity_X/maxDecel;
    }
    *pTTC_Brake=tBrake;
    *pTTC_Alert= tBrake + AEB_ALERT_BUFFER_TIME;
}

AEB_Mode_e AEB_ModeSelection(float ttc, float ttcBrake, float ttcAlert,
                             const AEB_Target_t *pAebTarget,
                             const EgoData_t *pEgoData)
{
    /* 설계서 2.2.6 => AEB 모드 결정 */
    if(!pAebTarget||!pEgoData) return AEB_MODE_NORMAL;
    if(pAebTarget->AEB_Target_ID<0) return AEB_MODE_NORMAL;
    if(pEgoData->Ego_Velocity_X<0.5f) return AEB_MODE_NORMAL;
    if(ttc<=0.0f || ttc>=99999.0f) return AEB_MODE_NORMAL;
    /* cut-out => normal */
    if(pAebTarget->AEB_Target_Situation==TGT_SITU_CUTOUT) {
        return AEB_MODE_NORMAL;
    }

    if(ttc> ttcAlert){
        return AEB_MODE_NORMAL;
    }
    else if(ttc> ttcBrake && ttc<= ttcAlert){
        return AEB_MODE_ALERT;
    }
    else if(ttc>0.0f && ttc<=ttcBrake){
        return AEB_MODE_BRAKE;
    }
    return AEB_MODE_NORMAL;
}

float AEB_CalcDecel(AEB_Mode_e aebMode,
                    float ttc,
                    float ttcBrake)
{
    if(aebMode==AEB_MODE_NORMAL||aebMode==AEB_MODE_ALERT){
        return 0.0f;
    }
    else if(aebMode==AEB_MODE_BRAKE){
        float ratio=1.0f - (ttc/(ttcBrake+1e-5f)); 
        float decel= AEB_MAX_BRAKE_DECEL * ratio;
        if(decel> AEB_MIN_BRAKE_DECEL) decel= AEB_MIN_BRAKE_DECEL; 
        if(decel< AEB_MAX_BRAKE_DECEL) decel= AEB_MAX_BRAKE_DECEL; 
        return decel;
    }
    return 0.0f;
}
