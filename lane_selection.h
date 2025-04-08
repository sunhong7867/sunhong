#ifndef LANE_SELECTION_H
#define LANE_SELECTION_H

#include "adas_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

int LaneSelection_Compute(const LaneData_t *pInLaneData,
                          const EgoData_t *pEgoData,
                          LaneSelectOutput_t *pLaneSelOut);

#ifdef __cplusplus
}
#endif

#endif /* LANE_SELECTION_H */
