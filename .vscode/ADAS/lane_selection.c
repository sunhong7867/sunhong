#include <math.h>
#include <string.h>
#include "lane_selection.h"

/*---------------------------------------------------------
 * LaneSelection_Compute
 * - 설계서 2.2.3
 *---------------------------------------------------------*/
int LaneSelection_Compute(const LaneData_t *pInLaneData,
                          const EgoData_t *pEgoData,
                          LaneSelectOutput_t *pLaneSelOut)
{
    if(!pInLaneData || !pEgoData || !pLaneSelOut) return -1;

    /* 1) 차선 유형 판단 (직선/곡선) */
    pLaneSelOut->LS_Lane_Type = pInLaneData->Lane_Type; 
    if(pInLaneData->Lane_Curvature < 1.0f) {
        /* 0 or 잘못된 값이면 일단 직선 처리 */
        pLaneSelOut->LS_Is_Curved_Lane = false;
    }
    else {
        /* 곡률 반경 < 800 => 곡선 */
        pLaneSelOut->LS_Is_Curved_Lane = 
            (pInLaneData->Lane_Curvature < LANE_CURVE_THRESHOLD);
    }

    /* 곡률 전이 */
    if(pInLaneData->Next_Lane_Curvature > 0.0f) {
        float diff = fabsf(pInLaneData->Next_Lane_Curvature - pInLaneData->Lane_Curvature);
        pLaneSelOut->LS_Curve_Transition_Flag = (diff > LANE_CURVE_DIFF_THRESHOLD);
    } else {
        pLaneSelOut->LS_Curve_Transition_Flag = false;
    }

    /* 2) Heading Error, Lane Offset */
    float headingDiff = pEgoData->Ego_Heading - pInLaneData->Lane_Heading;
    /* 정규화 */
    while(headingDiff>180.0f) headingDiff-=360.0f;
    while(headingDiff<-180.0f)headingDiff+=360.0f;

    pLaneSelOut->LS_Heading_Error = headingDiff;
    pLaneSelOut->LS_Lane_Offset   = pInLaneData->Lane_Offset; 
    pLaneSelOut->LS_Lane_Width    = pInLaneData->Lane_Width;

    /* 3) 차선 내 여부 판단 */
    float halfWidth = pInLaneData->Lane_Width * 0.5f;
    if(fabsf(pInLaneData->Lane_Offset) < halfWidth){
        pLaneSelOut->LS_Is_Within_Lane = true;
    } else {
        pLaneSelOut->LS_Is_Within_Lane = false;
    }

    /* 4) 차로 변경 상태 */
    if(pInLaneData->Lane_Change_Status == LANE_CHANGE_KEEP){
        pLaneSelOut->LS_Is_Changing_Lane = false;
    }
    else {
        pLaneSelOut->LS_Is_Changing_Lane = true;
    }

    return 0;
}
