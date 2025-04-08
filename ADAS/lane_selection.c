#include <math.h>
#include <string.h>
#include "lane_selection.h"

/*---------------------------------------------------------
 * LaneSelection_Update
 * - 설계서 2.2.3 "Lane Selection" 기능
 *---------------------------------------------------------*/
int LaneSelection_Update(const LaneData_t *pLaneData,
                         const EgoData_t  *pEgoData,
                         LaneSelectOutput_t *pLaneOut)
{
    if(!pLaneData || !pEgoData || !pLaneOut) {
        return -1; /* invalid argument */
    }

    /* 초기화 (출력 구조체) */
    memset(pLaneOut, 0, sizeof(LaneSelectOutput_t));

    /*------------------------------------------------------
     1) 차선 유형 판단 (직선 / 곡선) 및 곡률 전이
    ------------------------------------------------------*/
    /* Lane_Type 은 enum (직선, 곡선) */
    pLaneOut->LS_Lane_Type = pLaneData->Lane_Type; 
    /* 곡선 여부: 곡률 반경 < 800 => 곡선 */
    if(pLaneData->Lane_Curvature > 0.0f && pLaneData->Lane_Curvature < LANE_CURVE_THRESHOLD) {
        pLaneOut->LS_Is_Curved_Lane = true;
    } else {
        pLaneOut->LS_Is_Curved_Lane = false;
    }

    /* 곡률 전이 */
    if(pLaneData->Lane_Curvature > 0.0f && pLaneData->Next_Lane_Curvature > 0.0f) {
        float curvature_diff = fabsf(pLaneData->Next_Lane_Curvature - pLaneData->Lane_Curvature);
        if(curvature_diff > LANE_CURVE_DIFF_THRESHOLD) {
            pLaneOut->LS_Curve_Transition_Flag = true;
        } else {
            pLaneOut->LS_Curve_Transition_Flag = false;
        }
    } else {
        /* 0 이거나 유효X => 전이 판단 불가 => false */
        pLaneOut->LS_Curve_Transition_Flag = false;
    }

    /*------------------------------------------------------
     2) 진행 방향 오차, 차선 중심 오차
    ------------------------------------------------------*/
    /* heading_diff_raw = Ego_Heading - Lane_Heading */
    float heading_diff_raw = pEgoData->Ego_Heading - pLaneData->Lane_Heading;
    /* 정규화(±180) */
    while(heading_diff_raw >  180.0f) heading_diff_raw -= 360.0f;
    while(heading_diff_raw < -180.0f) heading_diff_raw += 360.0f;

    pLaneOut->LS_Heading_Error = heading_diff_raw;

    /* Lane Offset / Width */
    pLaneOut->LS_Lane_Offset = pLaneData->Lane_Offset; 
    pLaneOut->LS_Lane_Width  = pLaneData->Lane_Width;

    /* 차선 내 주행 여부: offset < laneWidth/2 ? */
    float offset_threshold = (pLaneData->Lane_Width * 0.5f);
    if(fabsf(pLaneData->Lane_Offset) < offset_threshold) {
        pLaneOut->LS_Is_Within_Lane = true;
    } else {
        pLaneOut->LS_Is_Within_Lane = false;
    }

    /*------------------------------------------------------
     3) 차로 변경 상태 판단
    ------------------------------------------------------*/
    /* Lane_Change_Status == 유지 -> false, 그 외 -> true */
    if(pLaneData->Lane_Change_Status == LANE_CHANGE_KEEP) {
        pLaneOut->LS_Is_Changing_Lane = false;
    } else {
        pLaneOut->LS_Is_Changing_Lane = true;
    }

    return 0;
}
