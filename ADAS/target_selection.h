#ifndef TARGET_SELECTION_H
#define TARGET_SELECTION_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 설계서 2.2.4 Target Selection 모듈 인터페이스
 * 1) select_target_from_object_list
 * 2) predict_object_future_path
 * 3) select_targets_for_acc_aeb
 */

/**
 * @brief select_target_from_object_list
 *        Carla로부터 수신된 ObjectData 리스트와 Ego/Lane 정보를 이용,
 *        유효 범위 내의 객체만 선별하고 상태 분류 및 셀 번호 부여 작업을 수행.
 *
 * @param[in]  pObjList        : 감지된 물체 리스트
 * @param[in]  objCount        : 물체 개수
 * @param[in]  pEgoData        : Ego 차량 상태
 * @param[in]  pLsData         : Lane Selection (LS) 결과
 * @param[out] pFilteredList   : 필터링된 객체 리스트
 * @param[in]  maxFilteredCount: pFilteredList에 담을 수 있는 최대 개수
 * @return 필터링 후 리스트에 저장된 객체 수
 */
int select_target_from_object_list(
    const ObjectData_t        *pObjList, 
    int                       objCount,
    const EgoData_t           *pEgoData,
    const LaneSelectOutput_t  *pLsData,
    FilteredObject_t          *pFilteredList, 
    int                       maxFilteredCount
);

/**
 * @brief predict_object_future_path
 *        필터링된 객체 리스트를 입력받아, 3초 후의 위치를 등속/등가속 모델로 예측.
 *        이후 Cut-in / Cut-out 판단을 수행해 PredictedObject_t 리스트를 생성.
 *
 * @param[in]  pFilteredList   : select_target_from_object_list에서 출력된 객체 리스트
 * @param[in]  filteredCount   : 필터링된 객체 수
 * @param[in]  pLaneWp         : (차량 waypoints, 필요 시 차선 곡률 등도 사용 가능)
 * @param[in]  pLsData         : Lane Selection 결과 (Lane 오프셋 등)
 * @param[out] pPredList       : 예측된 객체 리스트
 * @param[in]  maxPredCount    : pPredList 담을 수 있는 최대 개수
 * @return 예측된 객체 개수
 */
int predict_object_future_path(
    const FilteredObject_t    *pFilteredList, 
    int                       filteredCount,
    const LaneData_t          *pLaneWp,
    const LaneSelectOutput_t  *pLsData,
    PredictedObject_t         *pPredList, 
    int                       maxPredCount
);

/**
 * @brief select_targets_for_acc_aeb
 *        예측된 객체(최종 후보) 리스트 중 ACC, AEB 각각의 최우선 타겟을 선정.
 *
 * @param[in]  pEgoData    : Ego 차량 상태 (속도)
 * @param[in]  pPredList   : predict_object_future_path에서 출력된 객체 리스트
 * @param[in]  predCount   : 객체 개수
 * @param[in]  pLsData     : Lane Selection 결과 (곡선 여부 등)
 * @param[out] pAccTarget  : 선정된 ACC 타겟
 * @param[out] pAebTarget  : 선정된 AEB 타겟
 */
void select_targets_for_acc_aeb(
    const EgoData_t           *pEgoData,
    const PredictedObject_t   *pPredList, 
    int                       predCount,
    const LaneSelectOutput_t  *pLsData,
    ACC_Target_t              *pAccTarget,
    AEB_Target_t              *pAebTarget
);

#ifdef __cplusplus
}
#endif

#endif /* TARGET_SELECTION_H */
