#include <math.h>
#include <string.h>
#include <stdio.h>

#include "target_selection.h"

/* ----------------------------------------------------------------
 * 내부 유틸: heading 정규화 (±180°)
 * ---------------------------------------------------------------*/
static float normalize_heading(float hdg)
{
    while (hdg > 180.0f)   hdg -= 360.0f;
    while (hdg < -180.0f)  hdg += 360.0f;
    return hdg;
}

/*======================================================================
 * 1) select_target_from_object_list
 *    - 설계서 2.2.4.1.1
 *======================================================================*/
int select_target_from_object_list(const ObjectData_t *pObjList, 
                                   int                objCount,
                                   const EgoData_t   *pEgoData,
                                   const LaneSelectOutput_t *pLsData,
                                   FilteredObject_t  *pFilteredList, 
                                   int                maxFilteredCount)
{
    if (!pObjList || !pEgoData || !pLsData || !pFilteredList 
        || objCount <= 0 || maxFilteredCount <= 0) 
    {
        return 0;
    }

    int filteredIndex = 0;

    /* 곡선 차로 보정 계수 */
    float Heading_Error_Coeff = 0.05f;
    float Adjusted_Lateral_Threshold = pLsData->LS_Lane_Width;

    if (pLsData->LS_Is_Curved_Lane) {
        /* 곡선이면 차선 너비 + (fabs(Heading_Error) * 계수) */
        Adjusted_Lateral_Threshold += fabsf(pLsData->LS_Heading_Error) * Heading_Error_Coeff;
    }

    for (int i = 0; i < objCount; i++)
    {
        if (filteredIndex >= maxFilteredCount) 
            break;

        const ObjectData_t *obj = &pObjList[i];

        /* 1) 범위 필터링: 거리 200m 이하 */
        if (obj->Distance > 200.0f) {
            continue;
        }

        /* 2) 횡방향 필터링: Lateral Position = Obj.PositionY - LS_Lane_Offset */
        float Object_Lateral_Position = obj->Position_Y - pLsData->LS_Lane_Offset;
        if (fabsf(Object_Lateral_Position) > Adjusted_Lateral_Threshold) {
            continue;
        }

        /* 3) 상태 분류 */
        float Relative_Velocity = obj->Velocity_X - pEgoData->Ego_Velocity_X;
        float Heading_Difference = fabsf(obj->Heading - pEgoData->Ego_Heading);
        if (Heading_Difference > 180.0f) {
            Heading_Difference = 360.0f - Heading_Difference;
        }

        ObjectStatus_e finalStatus = obj->Object_Status; /* 우선은 입력된 값으로 초기 */

        /* Oncoming check */
        if (Heading_Difference >= 150.0f) {
            finalStatus = OBJSTAT_ONCOMING;
        }
        else {
            /* Moving vs. Stationary: |RelativeVel| >= 0.5 => Moving, else => Stationary */
            if (fabsf(Relative_Velocity) >= 0.5f) {
                finalStatus = OBJSTAT_MOVING;
            }
            else {
                /* 정밀하게 구분하려면 "이전 상태가 Moving이었으면 Stopped", ... 
                   여기서는 설계서에 "나머지는 Stationary"라고 단순 처리 */
                finalStatus = OBJSTAT_STATIONARY;
            }
        }

        /* 4) 곡선 차로 => 거리 보정 */
        float Adjusted_Object_Distance = obj->Distance;
        if (pLsData->LS_Is_Curved_Lane) {
            float he_rad = pLsData->LS_Heading_Error * (float)M_PI / 180.0f;
            float c = cosf(he_rad);
            if (fabsf(c) > 1.0e-3f) {
                Adjusted_Object_Distance = obj->Distance / c;
            }
        }

        /* 5) 셀 번호 부여 (Base_CellNumber) */
        int Base_CellNumber = 1;
        if (Adjusted_Object_Distance <= 60.0f) {
            Base_CellNumber = 1 + (int)(Adjusted_Object_Distance / 10.0f);
            if (Base_CellNumber > 6)  Base_CellNumber = 6;
        }
        else if (Adjusted_Object_Distance <= 120.0f) {
            float x = Adjusted_Object_Distance - 60.0f;
            Base_CellNumber = 7 + (int)(x / 10.0f);
            if (Base_CellNumber > 12) Base_CellNumber = 12;
        }
        else {
            float x = Adjusted_Object_Distance - 120.0f;
            Base_CellNumber = 13 + (int)(x / 10.0f);
            if (Base_CellNumber > 20) Base_CellNumber = 20;
        }

        /* 횡방향 위치 보정 offset => -1, 0, +1 */
        float Lane_Center_Offset = fabsf(obj->Position_Y - pLsData->LS_Lane_Offset);
        int   Offset_Adjustment   = 0;

        float quarterW = pLsData->LS_Lane_Width * 0.25f;
        float threeQW  = pLsData->LS_Lane_Width * 0.75f;
        if (Lane_Center_Offset < quarterW) {
            Offset_Adjustment = -1;
        }
        else if (Lane_Center_Offset >= threeQW) {
            Offset_Adjustment = +1;
        }

        int CellNumber = Base_CellNumber + Offset_Adjustment;
        if (CellNumber < 1)  CellNumber = 1;
        if (CellNumber > 20) CellNumber = 20;

        /* 최종 Filtered Object 구성 */
        FilteredObject_t *fObj = &pFilteredList[filteredIndex++];
        fObj->Filtered_Object_ID           = obj->Object_ID;
        fObj->Filtered_Object_Type         = obj->Object_Type;
        fObj->Filtered_Position_X          = obj->Position_X;
        fObj->Filtered_Position_Y          = obj->Position_Y;
        fObj->Filtered_Position_Z          = obj->Position_Z;
        fObj->Filtered_Velocity_X          = obj->Velocity_X;
        fObj->Filtered_Velocity_Y          = obj->Velocity_Y;
        fObj->Filtered_Accel_X             = obj->Accel_X;
        fObj->Filtered_Accel_Y             = obj->Accel_Y;
        fObj->Filtered_Heading             = normalize_heading(obj->Heading);
        fObj->Filtered_Distance            = Adjusted_Object_Distance;
        fObj->Filtered_Object_Status       = finalStatus;
        fObj->Filtered_Object_Cell_ID      = CellNumber;
    }

    return filteredIndex; /* 필터링된 객체 수 */
}

/*======================================================================
 * 2) predict_object_future_path
 *    - 설계서 2.2.4.1.2
 *======================================================================*/
int predict_object_future_path(const FilteredObject_t *pFilteredList, 
                               int                    filteredCount,
                               const LaneData_t      *pLaneWp,
                               const LaneSelectOutput_t *pLsData,
                               PredictedObject_t     *pPredList, 
                               int                    maxPredCount)
{
    if (!pFilteredList || !pLaneWp || !pLsData || !pPredList 
        || filteredCount <= 0 || maxPredCount <= 0) 
    {
        return 0;
    }
    int predIndex = 0;
    float t_predict = 3.0f;  /* 3초 예측 시간 */

    for (int i = 0; i < filteredCount; i++)
    {
        if (predIndex >= maxPredCount) break;

        const FilteredObject_t *fo = &pFilteredList[i];

        /* 새로운 PredictedObject 생성 */
        PredictedObject_t *po = &pPredList[predIndex++];
        memset(po, 0, sizeof(PredictedObject_t));

        po->Predicted_Object_ID       = fo->Filtered_Object_ID;
        po->Predicted_Object_Type     = fo->Filtered_Object_Type;
        po->Predicted_Heading         = fo->Filtered_Heading;
        po->Predicted_Object_Status   = fo->Filtered_Object_Status;
        po->Predicted_Object_Cell_ID  = fo->Filtered_Object_Cell_ID;

        /* 속도/가속도 */
        float vx = fo->Filtered_Velocity_X;
        float vy = fo->Filtered_Velocity_Y;
        float ax = fo->Filtered_Accel_X;
        float ay = fo->Filtered_Accel_Y;

        /* 초기 위치 */
        float x0 = fo->Filtered_Position_X;
        float y0 = fo->Filtered_Position_Y;

        /* Moving => 등속, Stopped/감속 => 등가속 */
        if (fo->Filtered_Object_Status == OBJSTAT_MOVING)
        {
            po->Predicted_Position_X = x0 + vx * t_predict;
            po->Predicted_Position_Y = y0 + vy * t_predict;
        }
        else
        {
            po->Predicted_Position_X = x0 + vx * t_predict 
                                       + 0.5f * ax * (t_predict * t_predict);
            po->Predicted_Position_Y = y0 + vy * t_predict 
                                       + 0.5f * ay * (t_predict * t_predict);
        }

        po->Predicted_Position_Z = fo->Filtered_Position_Z;

        po->Predicted_Velocity_X = vx;
        po->Predicted_Velocity_Y = vy;
        po->Predicted_Accel_X    = ax;
        po->Predicted_Accel_Y    = ay;

        /* 거리 재계산 */
        float dx = po->Predicted_Position_X;
        float dy = po->Predicted_Position_Y;
        float dist = sqrtf(dx*dx + dy*dy);
        po->Predicted_Distance = dist;

        /* CutIn_Flag, CutOut_Flag 판단 */
        po->CutIn_Flag  = false;
        po->CutOut_Flag = false;

        {
            float Object_Lateral_Position = po->Predicted_Position_Y - pLsData->LS_Lane_Offset;
            float CutIn_Threshold = 0.85f;
            float Ego_Lane_Boundary = pLsData->LS_Lane_Width * 0.5f;

            /* Cut-in */
            if ((vx >= 0.5f) && (fabsf(vy) >= 0.2f) 
                 && (fabsf(Object_Lateral_Position) <= CutIn_Threshold))
            {
                po->CutIn_Flag = true;
            }
            /* Cut-out */
            if ((fabsf(vy) >= 0.2f) 
                 && (fabsf(Object_Lateral_Position) > (Ego_Lane_Boundary + CutIn_Threshold)))
            {
                po->CutOut_Flag = true;
            }
        }
    }

    return predIndex; 
}

/*======================================================================
 * 3) select_targets_for_acc_aeb
 *    - 설계서 2.2.4.1.3
 *======================================================================*/
void select_targets_for_acc_aeb(const EgoData_t *pEgoData,
                                const PredictedObject_t *pPredList, 
                                int predCount,
                                const LaneSelectOutput_t *pLsData,
                                ACC_Target_t  *pAccTarget,
                                AEB_Target_t  *pAebTarget)
{
    if (!pEgoData || !pPredList || !pLsData 
        || !pAccTarget || !pAebTarget || predCount <= 0)
    {
        /* 타겟 유효 X */
        if (pAccTarget) pAccTarget->ACC_Target_ID = -1;
        if (pAebTarget) pAebTarget->AEB_Target_ID = -1;
        return;
    }

    /* 초기값 */
    pAccTarget->ACC_Target_ID = -1;
    pAebTarget->AEB_Target_ID = -1;

    /* 임의로 Situation (Normal, Cut-in, etc.) 를 enum/define… */
    pAccTarget->ACC_Target_Situation = TGT_SITU_NORMAL;
    pAebTarget->AEB_Target_Situation = TGT_SITU_NORMAL;

    float bestAccScore = -999999.0f;
    int   bestAccIdx   = -1;

    float bestAebScore = -999999.0f;
    int   bestAebIdx   = -1;

    /* Brake_Status: Ego 속도가 매우 작으면 (정지 가정) */
    bool Brake_Status = (fabsf(pEgoData->Ego_Velocity_X) < 0.1f);

    /* 우선순위 평가 */
    for (int i = 0; i < predCount; i++)
    {
        const PredictedObject_t *obj = &pPredList[i];

        /* Cut-out 제외 */
        if (obj->CutOut_Flag) {
            continue;
        }
        float px = obj->Predicted_Position_X;
        float py = obj->Predicted_Position_Y;

        if (px < 0.0f) {
            /* 후방 => skip */
            continue;
        }

        /*=== ACC 후보 조건 ===*/
        /* 정면( |y|<=1.75 ), 타입=car, 상태=Moving/Stopped, cutOut=false */
        if ((fabsf(py) <= 1.75f) 
            && (obj->Predicted_Object_Type == OBJTYPE_CAR)
            && ((obj->Predicted_Object_Status == OBJSTAT_MOVING)
                ||(obj->Predicted_Object_Status == OBJSTAT_STOPPED)))
        {
            float dist = obj->Predicted_Distance;
            /* 점수 = 200-dist + 곡선 추가 보정 */
            float score = 200.0f - dist;
            if (pLsData->LS_Is_Curved_Lane 
                && obj->Predicted_Object_Cell_ID < 5) {
                score += 10.0f; 
            }
            if (score > bestAccScore) {
                bestAccScore = score;
                bestAccIdx = i;
            }
        }

        /*=== AEB 후보 조건 ===*/
        bool isFront = (fabsf(py) <= 1.75f);
        bool isSide  = ((fabsf(py) > 1.75f) && (fabsf(py) <= 3.5f));
        bool aebCandidate = false;

        if (isFront) {
            /* front + {Moving,Stopped} OR (Stationary & brake_status==true) */
            if (obj->Predicted_Object_Status == OBJSTAT_MOVING 
             || obj->Predicted_Object_Status == OBJSTAT_STOPPED) {
                aebCandidate = true;
            }
            else if ((obj->Predicted_Object_Status == OBJSTAT_STATIONARY) 
                      && Brake_Status) {
                aebCandidate = true;
            }
        }
        else if (isSide) {
            /* 측면 + cutin => AEB 대상 */
            if (obj->CutIn_Flag) aebCandidate = true;
        }

        if (aebCandidate)
        {
            /* TTC 판단 */
            float relSpeed = pEgoData->Ego_Velocity_X - obj->Predicted_Velocity_X;
            float ttc = 999999.0f;
            if (relSpeed > 0.1f) {
                ttc = (obj->Predicted_Distance / relSpeed);
            }
            /* 점수 = 200-dist + cutin bonus + ttc<3 => +20 */
            float score = 200.0f - obj->Predicted_Distance;
            if (obj->CutIn_Flag) {
                score += 30.0f; 
            }
            if (ttc < 3.0f) {
                score += 20.0f;
            }

            if (score > bestAebScore) {
                bestAebScore = score;
                bestAebIdx = i;
            }
        }
    }

    /*=== ACC 최종 타겟 ===*/
    if (bestAccIdx >= 0) {
        const PredictedObject_t *obj = &pPredList[bestAccIdx];
        pAccTarget->ACC_Target_ID         = obj->Predicted_Object_ID;
        pAccTarget->ACC_Target_Position_X = obj->Predicted_Position_X;
        pAccTarget->ACC_Target_Position_Y = obj->Predicted_Position_Y;
        pAccTarget->ACC_Target_Vel_X      = obj->Predicted_Velocity_X;
        pAccTarget->ACC_Target_Vel_Y      = obj->Predicted_Velocity_Y;
        pAccTarget->ACC_Target_Accel_X    = obj->Predicted_Accel_X;
        pAccTarget->ACC_Target_Accel_Y    = obj->Predicted_Accel_Y;
        pAccTarget->ACC_Target_Distance   = obj->Predicted_Distance;
        pAccTarget->ACC_Target_Heading    = obj->Predicted_Heading;
        pAccTarget->ACC_Target_Status     = obj->Predicted_Object_Status;

        /* 상황 (Cut-in/out/Normal etc.) */
        if (obj->CutIn_Flag) 
            pAccTarget->ACC_Target_Situation = TGT_SITU_CUTIN;
        else if (obj->CutOut_Flag) 
            pAccTarget->ACC_Target_Situation = TGT_SITU_CUTOUT;
        else if (pLsData->LS_Is_Curved_Lane) 
            pAccTarget->ACC_Target_Situation = TGT_SITU_CURVE; /* 예시 */
        else 
            pAccTarget->ACC_Target_Situation = TGT_SITU_NORMAL;
    }

    /*=== AEB 최종 타겟 ===*/
    if (bestAebIdx >= 0) {
        const PredictedObject_t *obj = &pPredList[bestAebIdx];
        pAebTarget->AEB_Target_ID         = obj->Predicted_Object_ID;
        pAebTarget->AEB_Target_Position_X = obj->Predicted_Position_X;
        pAebTarget->AEB_Target_Position_Y = obj->Predicted_Position_Y;
        pAebTarget->AEB_Target_Vel_X      = obj->Predicted_Velocity_X;
        pAebTarget->AEB_Target_Vel_Y      = obj->Predicted_Velocity_Y;
        pAebTarget->AEB_Target_Accel_X    = obj->Predicted_Accel_X;
        pAebTarget->AEB_Target_Accel_Y    = obj->Predicted_Accel_Y;
        pAebTarget->AEB_Target_Distance   = obj->Predicted_Distance;
        pAebTarget->AEB_Target_Heading    = obj->Predicted_Heading;
        pAebTarget->AEB_Target_Status     = obj->Predicted_Object_Status;

        if (obj->CutIn_Flag) 
            pAebTarget->AEB_Target_Situation = TGT_SITU_CUTIN;
        else if (obj->CutOut_Flag) 
            pAebTarget->AEB_Target_Situation = TGT_SITU_CUTOUT;
        else if (pLsData->LS_Is_Curved_Lane) 
            pAebTarget->AEB_Target_Situation = TGT_SITU_CURVE;
        else 
            pAebTarget->AEB_Target_Situation = TGT_SITU_NORMAL;
    }
}
