#ifndef TARGET_SELECTION_H
#define TARGET_SELECTION_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 1) select_target_from_object_list */
int select_target_from_object_list(const ObjectData_t *pObjList, int objCount,
                                   const EgoData_t *pEgoData,
                                   const LaneSelectOutput_t *pLsData,
                                   FilteredObject_t *pFilteredList, int maxFilteredCount);

/* 2) predict_object_future_path */
int predict_object_future_path(const FilteredObject_t *pFilteredList, int filteredCount,
                               const LaneData_t *pLaneWp, 
                               const LaneSelectOutput_t *pLsData,
                               PredictedObject_t *pPredList, int maxPredCount);

/* 3) select_targets_for_acc_aeb */
void select_targets_for_acc_aeb(const EgoData_t *pEgoData,
                                const PredictedObject_t *pPredList, int predCount,
                                const LaneSelectOutput_t *pLsData,
                                ACC_Target_t *pAccTarget,
                                AEB_Target_t *pAebTarget);

#ifdef __cplusplus
}
#endif

#endif /* TARGET_SELECTION_H */
