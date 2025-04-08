#include <math.h>
#include <string.h>
#include <stdio.h>
#include "target_selection.h"

static float normalize_heading(float hdg)
{
    while(hdg > 180.0f)  hdg -= 360.0f;
    while(hdg < -180.0f) hdg += 360.0f;
    return hdg;
}

/* 1) select_target_from_object_list */
int select_target_from_object_list(const ObjectData_t *pObjList, int objCount,
                                   const EgoData_t *pEgoData,
                                   const LaneSelectOutput_t *pLsData,
                                   FilteredObject_t *pFilteredList, int maxFilteredCount)
{
    if(!pObjList || !pEgoData || !pLsData || !pFilteredList) return 0;
    int filteredIndex = 0;

    float heading_coeff = 0.05f;
    float adjustedLat = pLsData->LS_Lane_Width;
    if(pLsData->LS_Is_Curved_Lane){
        adjustedLat += fabsf(pLsData->LS_Heading_Error)*heading_coeff;
    }

    for(int i=0; i<objCount; i++){
        if(filteredIndex>=maxFilteredCount) break;
        const ObjectData_t *obj = &pObjList[i];
        if(obj->Distance>200.0f) continue; /* 200m 초과 제외 */

        float lateralPos = obj->Position_Y - pLsData->LS_Lane_Offset;
        if(fabsf(lateralPos)>adjustedLat) continue; /* 횡방향 제외 */

        /* 상태 분류 */
        float relVel = obj->Velocity_X - pEgoData->Ego_Velocity_X;
        float hdiff = fabsf(obj->Heading - pEgoData->Ego_Heading);
        if(hdiff>180.0f) hdiff=360.0f-hdiff;

        ObjectStatus_e finalStat = obj->Object_Status;
        if(hdiff>=150.0f){
            finalStat=OBJSTAT_ONCOMING;
        } else {
            if(fabsf(relVel)>=0.5f){
                finalStat=OBJSTAT_MOVING;
            } else {
                finalStat=OBJSTAT_STATIONARY;
            }
        }

        /* 곡선 -> dist 보정 */
        float adjDist=obj->Distance;
        if(pLsData->LS_Is_Curved_Lane){
            float hrad = pLsData->LS_Heading_Error*(float)M_PI/180.0f;
            float c=cosf(hrad);
            if(fabsf(c)>1e-3f){
                adjDist=obj->Distance/c;
            }
        }

        /* 셀 번호 */
        int baseCell=1;
        if(adjDist<=60.0f){
            baseCell=1+(int)(adjDist/10.0f); if(baseCell>6) baseCell=6;
        } else if(adjDist<=120.0f){
            float x=adjDist-60.0f;
            baseCell=7+(int)(x/10.0f); if(baseCell>12) baseCell=12;
        } else {
            float x=adjDist-120.0f;
            baseCell=13+(int)(x/10.0f); if(baseCell>20) baseCell=20;
        }

        float laneCenterOffset = fabsf(obj->Position_Y-pLsData->LS_Lane_Offset);
        int offsetAdjust=0;
        float qW = pLsData->LS_Lane_Width*0.25f;
        float tqW= pLsData->LS_Lane_Width*0.75f;
        if(laneCenterOffset<qW) offsetAdjust=-1;
        else if(laneCenterOffset>=tqW) offsetAdjust=+1;

        int cellNum=baseCell+offsetAdjust;
        if(cellNum<1) cellNum=1;
        if(cellNum>20)cellNum=20;

        FilteredObject_t *fo = &pFilteredList[filteredIndex++];
        fo->Filtered_Object_ID = obj->Object_ID;
        fo->Filtered_Object_Type=obj->Object_Type;
        fo->Filtered_Position_X=obj->Position_X;
        fo->Filtered_Position_Y=obj->Position_Y;
        fo->Filtered_Position_Z=obj->Position_Z;
        fo->Filtered_Velocity_X=obj->Velocity_X;
        fo->Filtered_Velocity_Y=obj->Velocity_Y;
        fo->Filtered_Accel_X   =obj->Accel_X;
        fo->Filtered_Accel_Y   =obj->Accel_Y;
        fo->Filtered_Heading   = normalize_heading(obj->Heading);
        fo->Filtered_Distance  = adjDist;
        fo->Filtered_Object_Status=finalStat;
        fo->Filtered_Object_Cell_ID=cellNum;
    }

    return filteredIndex;
}

/* 2) predict_object_future_path */
int predict_object_future_path(const FilteredObject_t *pFilteredList, int filteredCount,
                               const LaneData_t *pLaneWp, 
                               const LaneSelectOutput_t *pLsData,
                               PredictedObject_t *pPredList, int maxPredCount)
{
    if(!pFilteredList||!pLaneWp||!pLsData||!pPredList) return 0;
    int predIndex=0;
    float t_predict=3.0f;

    for(int i=0; i<filteredCount; i++){
        if(predIndex>=maxPredCount) break;

        const FilteredObject_t *fo = &pFilteredList[i];
        PredictedObject_t *po=&pPredList[predIndex++];
        memset(po,0,sizeof(PredictedObject_t));

        po->Predicted_Object_ID   =fo->Filtered_Object_ID;
        po->Predicted_Object_Type =fo->Filtered_Object_Type;
        po->Predicted_Heading     =fo->Filtered_Heading;
        po->Predicted_Object_Status=fo->Filtered_Object_Status;
        po->Predicted_Object_Cell_ID=fo->Filtered_Object_Cell_ID;

        float vx= fo->Filtered_Velocity_X;
        float vy= fo->Filtered_Velocity_Y;
        float ax= fo->Filtered_Accel_X;
        float ay= fo->Filtered_Accel_Y;
        float x0= fo->Filtered_Position_X;
        float y0= fo->Filtered_Position_Y;

        if(fo->Filtered_Object_Status==OBJSTAT_MOVING){
            po->Predicted_Position_X=x0+vx*t_predict;
            po->Predicted_Position_Y=y0+vy*t_predict;
        } else {
            po->Predicted_Position_X=x0+vx*t_predict+0.5f*ax*(t_predict*t_predict);
            po->Predicted_Position_Y=y0+vy*t_predict+0.5f*ay*(t_predict*t_predict);
        }
        po->Predicted_Position_Z=fo->Filtered_Position_Z;
        po->Predicted_Velocity_X=vx;
        po->Predicted_Velocity_Y=vy;
        po->Predicted_Accel_X   =ax;
        po->Predicted_Accel_Y   =ay;

        float dx=po->Predicted_Position_X;
        float dy=po->Predicted_Position_Y;
        float dist=sqrtf(dx*dx+dy*dy);
        po->Predicted_Distance=dist;

        /* cut-in / cut-out */
        po->CutIn_Flag=false;
        po->CutOut_Flag=false;
        {
            float lateralPos = po->Predicted_Position_Y - pLsData->LS_Lane_Offset;
            float cutInTh=0.85f;
            float laneBoundary= pLsData->LS_Lane_Width*0.5f;
            if((vx>=0.5f)&&(fabsf(vy)>=0.2f)&&(fabsf(lateralPos)<=cutInTh)){
                po->CutIn_Flag=true;
            }
            if((fabsf(vy)>=0.2f)&&(fabsf(lateralPos)>(laneBoundary+cutInTh))){
                po->CutOut_Flag=true;
            }
        }
    }
    return predIndex;
}

/* 3) select_targets_for_acc_aeb */
void select_targets_for_acc_aeb(const EgoData_t *pEgoData,
                                const PredictedObject_t *pPredList, int predCount,
                                const LaneSelectOutput_t *pLsData,
                                ACC_Target_t *pAccTarget,
                                AEB_Target_t *pAebTarget)
{
    if(!pEgoData||!pPredList||!pLsData||!pAccTarget||!pAebTarget) {
        return;
    }
    pAccTarget->ACC_Target_ID=-1;
    pAebTarget->AEB_Target_ID=-1;
    pAccTarget->ACC_Target_Situation=TGT_SITU_NORMAL;
    pAebTarget->AEB_Target_Situation=TGT_SITU_NORMAL;

    float bestAccScore=-9999.0f;
    int bestAccIdx=-1;

    float bestAebScore=-9999.0f;
    int bestAebIdx=-1;

    bool egoStopped= (fabsf(pEgoData->Ego_Velocity_X)<0.1f);

    for(int i=0;i<predCount;i++){
        const PredictedObject_t *obj=&pPredList[i];
        if(obj->CutOut_Flag) continue;
        float x= obj->Predicted_Position_X;
        float y= obj->Predicted_Position_Y;
        if(x<0.0f) continue; /* 후방 제외 */

        /* ACC 후보 */
        if((fabsf(y)<=1.75f) &&
           (obj->Predicted_Object_Type==OBJTYPE_CAR) &&
           ((obj->Predicted_Object_Status==OBJSTAT_MOVING)||(obj->Predicted_Object_Status==OBJSTAT_STOPPED)))
        {
            float dist=obj->Predicted_Distance;
            float score=200.0f-dist;
            if(pLsData->LS_Is_Curved_Lane && obj->Predicted_Object_Cell_ID<5) score+=10.0f;
            if(score>bestAccScore){
                bestAccScore=score; 
                bestAccIdx=i;
            }
        }

        /* AEB 후보 */
        bool isFront=(fabsf(y)<=1.75f);
        bool isSide =(fabsf(y)>1.75f && fabsf(y)<=3.5f);
        bool aebCandi=false;
        if(isFront){
            if(obj->Predicted_Object_Status==OBJSTAT_MOVING ||
               obj->Predicted_Object_Status==OBJSTAT_STOPPED){
                aebCandi=true;
            }
            else if(obj->Predicted_Object_Status==OBJSTAT_STATIONARY && egoStopped){
                aebCandi=true;
            }
        } else if(isSide && obj->CutIn_Flag){
            aebCandi=true;
        }
        if(aebCandi){
            float relSpeed = pEgoData->Ego_Velocity_X - obj->Predicted_Velocity_X;
            float ttc=99999.0f;
            if(relSpeed>0.1f){
                ttc=obj->Predicted_Distance/relSpeed;
            }
            float score=200.0f - obj->Predicted_Distance;
            if(obj->CutIn_Flag) score+=30.0f;
            if(ttc<3.0f) score+=20.0f;
            if(score>bestAebScore){
                bestAebScore=score; 
                bestAebIdx=i;
            }
        }
    }

    /* ACC */
    if(bestAccIdx>=0){
        const PredictedObject_t *obj=&pPredList[bestAccIdx];
        pAccTarget->ACC_Target_ID        = obj->Predicted_Object_ID;
        pAccTarget->ACC_Target_Position_X= obj->Predicted_Position_X;
        pAccTarget->ACC_Target_Position_Y= obj->Predicted_Position_Y;
        pAccTarget->ACC_Target_Vel_X     = obj->Predicted_Velocity_X;
        pAccTarget->ACC_Target_Vel_Y     = obj->Predicted_Velocity_Y;
        pAccTarget->ACC_Target_Accel_X   = obj->Predicted_Accel_X;
        pAccTarget->ACC_Target_Accel_Y   = obj->Predicted_Accel_Y;
        pAccTarget->ACC_Target_Distance  = obj->Predicted_Distance;
        pAccTarget->ACC_Target_Heading   = obj->Predicted_Heading;
        pAccTarget->ACC_Target_Status    = obj->Predicted_Object_Status;

        /* 상황: cutin/out/curve */
        if(obj->CutIn_Flag) pAccTarget->ACC_Target_Situation = TGT_SITU_CUTIN;
        else if(obj->CutOut_Flag) pAccTarget->ACC_Target_Situation= TGT_SITU_CUTOUT;
        else if(pLsData->LS_Is_Curved_Lane) pAccTarget->ACC_Target_Situation=TGT_SITU_CURVE;
        else pAccTarget->ACC_Target_Situation = TGT_SITU_NORMAL;
    }

    /* AEB */
    if(bestAebIdx>=0){
        const PredictedObject_t *obj=&pPredList[bestAebIdx];
        pAebTarget->AEB_Target_ID        = obj->Predicted_Object_ID;
        pAebTarget->AEB_Target_Position_X= obj->Predicted_Position_X;
        pAebTarget->AEB_Target_Position_Y= obj->Predicted_Position_Y;
        pAebTarget->AEB_Target_Vel_X     = obj->Predicted_Velocity_X;
        pAebTarget->AEB_Target_Vel_Y     = obj->Predicted_Velocity_Y;
        pAebTarget->AEB_Target_Accel_X   = obj->Predicted_Accel_X;
        pAebTarget->AEB_Target_Accel_Y   = obj->Predicted_Accel_Y;
        pAebTarget->AEB_Target_Distance  = obj->Predicted_Distance;
        pAebTarget->AEB_Target_Heading   = obj->Predicted_Heading;
        pAebTarget->AEB_Target_Status    = obj->Predicted_Object_Status;

        /* cutin/out/curve */
        if(obj->CutIn_Flag) pAebTarget->AEB_Target_Situation = TGT_SITU_CUTIN;
        else if(obj->CutOut_Flag) pAebTarget->AEB_Target_Situation= TGT_SITU_CUTOUT;
        else if(pLsData->LS_Is_Curved_Lane) pAebTarget->AEB_Target_Situation=TGT_SITU_CURVE;
        else pAebTarget->AEB_Target_Situation = TGT_SITU_NORMAL;
    }
}
