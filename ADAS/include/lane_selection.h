#ifndef LANE_SELECTION_H
#define LANE_SELECTION_H

#include "adas_shared.h"  /* LaneData_t, EgoData_t, LaneSelectOutput_t, etc. */

#ifdef __cplusplus
extern "C" {
#endif

/*
  설계서 2.2.3 "Lane Selection" 모듈 함수 프로토타입
  - 입력: LaneData_t, EgoData_t
  - 출력: LaneSelectOutput_t
*/

/**
 * @brief LaneSelection_Update
 *        설계서에 정의된 Lane Selection 기능 수행:
 *        1) 차선 유형(직선/곡선) 판단
 *        2) 곡률 전이 플래그
 *        3) Heading Error / Lane Offset 계산
 *        4) 차선 변경 상태 판단
 *
 * @param[in]  pLaneData  : Lane Data (차선 곡률, 오프셋, 폭, 등)
 * @param[in]  pEgoData   : Ego 차량 (Heading, Position=0, etc.)
 * @param[out] pLaneOut   : Lane Selection 결과 (LS_Is_Curved_Lane, LS_Heading_Error, ...)
 *
 * @return 0 on success, negative on error
 */
int LaneSelection_Update(const LaneData_t *pLaneData,
                         const EgoData_t  *pEgoData,
                         LaneSelectOutput_t *pLaneOut);

#ifdef __cplusplus
}
#endif

#endif /* LANE_SELECTION_H */
