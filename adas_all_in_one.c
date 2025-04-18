/****************************************************************************
 * adas_all_in_one.c
 *
 * 모든 모듈(ego estimation, lane selection, target selection, ACC, AEB, LFA, arbitration)
 * 을 하나의 소스 파일로 통합한 예시
 ****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/*============================================================================
 * 1) adas_shared.h : 공통 자료형 및 상수 정의
 *============================================================================*/

/* M_PI가 math.h에 없을 경우 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 1-1. Time, GPS, IMU, EgoData */
typedef struct {
    float Current_Time;  /* [ms] 제어루프 시각 */
} TimeData_t;

typedef struct {
    float GPS_Velocity_X;
    float GPS_Velocity_Y;
    float GPS_Timestamp;
} GPSData_t;

typedef struct {
    float Linear_Acceleration_X;
    float Linear_Acceleration_Y;
    float Yaw_Rate;  /* [deg/s] */
} IMUData_t;

typedef struct {
    float Ego_Velocity_X;      /* [m/s] */
    float Ego_Velocity_Y;      /* [m/s] */
    float Ego_Acceleration_X;  /* [m/s^2] */
    float Ego_Acceleration_Y;  /* [m/s^2] */
    float Ego_Heading;         /* [deg], -180~180 */
    float Ego_Yaw_Rate;        /* [deg/s] */
    
    float Ego_Position_X;
    float Ego_Position_Y;
    float Ego_Position_Z;
} EgoData_t;

/* 1-2. Lane Data & Lane Selection Output */
typedef enum {
    LANE_TYPE_STRAIGHT = 0,
    LANE_TYPE_CURVE
} LaneType_e;

typedef enum {
    LANE_CHANGE_KEEP = 0,
    LANE_CHANGE_CHANGING,
    LANE_CHANGE_DONE
} LaneChangeStatus_e;

typedef struct {
    LaneType_e  Lane_Type;
    float       Lane_Curvature;      
    float       Next_Lane_Curvature; 
    float       Lane_Offset;         
    float       Lane_Heading;        
    float       Lane_Width;          
    LaneChangeStatus_e Lane_Change_Status;
} LaneData_t;

typedef struct {
    LaneType_e  LS_Lane_Type;            
    bool        LS_Is_Curved_Lane;       
    bool        LS_Curve_Transition_Flag;
    float       LS_Heading_Error;        
    float       LS_Lane_Offset;          
    float       LS_Lane_Width;           
    bool        LS_Is_Within_Lane;       
    bool        LS_Is_Changing_Lane;
} LaneSelectOutput_t;

#define LANE_CURVE_THRESHOLD       800.0f
#define LANE_CURVE_DIFF_THRESHOLD  400.0f

/* 1-3. Target Selection 관련 자료형 */
typedef enum {
    OBJTYPE_CAR = 0,
    OBJTYPE_PEDESTRIAN,
    OBJTYPE_BICYCLE,
    OBJTYPE_MOTORCYCLE
} ObjectType_e;

typedef enum {
    OBJSTAT_MOVING = 0,
    OBJSTAT_STOPPED,
    OBJSTAT_STATIONARY,
    OBJSTAT_ONCOMING
} ObjectStatus_e;

typedef struct {
    int   Object_ID;
    ObjectType_e Object_Type;
    float Position_X;
    float Position_Y;
    float Position_Z;
    float Velocity_X;
    float Velocity_Y;
    float Accel_X;
    float Accel_Y;
    float Heading;
    float Distance;
    ObjectStatus_e Object_Status;
    int   Object_Cell_ID;
} ObjectData_t;

typedef struct {
    int   Filtered_Object_ID;
    ObjectType_e Filtered_Object_Type;
    float Filtered_Position_X;
    float Filtered_Position_Y;
    float Filtered_Position_Z;
    float Filtered_Velocity_X;
    float Filtered_Velocity_Y;
    float Filtered_Accel_X;
    float Filtered_Accel_Y;
    float Filtered_Heading;
    float Filtered_Distance;
    ObjectStatus_e Filtered_Object_Status;
    int   Filtered_Object_Cell_ID;
} FilteredObject_t;

typedef struct {
    int   Predicted_Object_ID;
    ObjectType_e Predicted_Object_Type;
    float Predicted_Position_X;
    float Predicted_Position_Y;
    float Predicted_Position_Z;
    float Predicted_Velocity_X;
    float Predicted_Velocity_Y;
    float Predicted_Accel_X;
    float Predicted_Accel_Y;
    float Predicted_Heading;
    float Predicted_Distance;
    ObjectStatus_e Predicted_Object_Status;
    int   Predicted_Object_Cell_ID;
    
    bool  CutIn_Flag;
    bool  CutOut_Flag;
} PredictedObject_t;

#define MAX_OBJECT_DISTANCE  200.0f
#define LATERAL_THRESHOLD    4.0f  /* ±4m */

/* 타겟 상황: Normal, Cut-in, Cut-out, Curve */
typedef enum {
    TGT_SITU_NORMAL = 0,
    TGT_SITU_CUTIN,
    TGT_SITU_CUTOUT,
    TGT_SITU_CURVE
} TargetSituation_e;

/* 1-4. ACC / AEB 타겟 */
typedef struct {
    int   ACC_Target_ID;
    float ACC_Target_Position_X;
    float ACC_Target_Position_Y;
    float ACC_Target_Vel_X;
    float ACC_Target_Vel_Y;
    float ACC_Target_Accel_X;
    float ACC_Target_Accel_Y;
    float ACC_Target_Distance;
    float ACC_Target_Heading;
    ObjectStatus_e   ACC_Target_Status;
    TargetSituation_e ACC_Target_Situation;
} ACC_Target_t;

typedef struct {
    int   AEB_Target_ID;
    float AEB_Target_Position_X;
    float AEB_Target_Position_Y;
    float AEB_Target_Vel_X;
    float AEB_Target_Vel_Y;
    float AEB_Target_Accel_X;
    float AEB_Target_Accel_Y;
    float AEB_Target_Distance;
    float AEB_Target_Heading;
    ObjectStatus_e   AEB_Target_Status;
    TargetSituation_e AEB_Target_Situation;
} AEB_Target_t;

/* 1-5. ACC/AEB 모드 및 상수 */
typedef enum {
    ACC_MODE_SPEED = 0,
    ACC_MODE_DISTANCE,
    ACC_MODE_STOP
} ACC_Mode_e;

#define MAX_ACCEL  10.0f
#define MIN_ACCEL -10.0f

typedef enum {
    AEB_MODE_NORMAL = 0,
    AEB_MODE_ALERT,
    AEB_MODE_BRAKE
} AEB_Mode_e;

#define AEB_MAX_BRAKE_DECEL  -10.0f
#define AEB_MIN_BRAKE_DECEL  -2.0f
#define AEB_MAX_BRAKE_SPEED   0.5f
#define AEB_ALERT_BUFFER_TIME 1.2f
#define AEB_DEFAULT_MAX_DECEL 9.0f

/* 1-6. LFA 모드 */
typedef enum {
    LFA_MODE_LOW_SPEED = 0,
    LFA_MODE_HIGH_SPEED
} LFA_Mode_e;

#define LFA_LOW_SPEED_THRESHOLD 16.67f
#define LFA_MAX_STEERING_ANGLE  540.0f

/*============================================================================
 * 2) Ego Vehicle Estimation Module
 *    (칼만 필터를 이용하여 IMU+GPS 데이터로 EgoData 추정)
 *============================================================================*/
typedef struct {
    float X[5];  /* 상태 벡터: [vx, vy, ax, ay, heading] */
    float P[25]; /* 5x5 공분산행렬 */

    float Last_GPS_VelX, Last_GPS_VelY;
    float Last_GPS_Timestamp;
    float Prev_Update_Time;

    float Prev_Accel_X;
    float Prev_Accel_Y;
    float Prev_YawRate;
    float Prev_GPS_VelX;
    float Prev_GPS_VelY;
} EgoEstimationKFState_t;

#define MAX_SENSOR_NOISE_ACCEL   3.0f
#define MAX_SENSOR_NOISE_YAWRATE 30.0f
#define MAX_SENSOR_NOISE_GPSVEL  10.0f
#define GPS_VALID_TIME_THRESH    50.0f

static bool EgoEst_CheckSpike(float newVal, float oldVal, float threshold)
{
    return (fabsf(newVal - oldVal) > threshold);
}

void EgoEstimation_Init(EgoEstimationKFState_t *pState)
{
    if(!pState) return;
    memset(pState, 0, sizeof(*pState));
}

void EgoEstimation_Update(
    const TimeData_t *timeData,
    const GPSData_t  *gpsData,
    const IMUData_t  *imuData,
    EgoData_t        *egoData,
    EgoEstimationKFState_t *pState
)
{
    if(!timeData || !gpsData || !imuData || !egoData || !pState)
        return;

    /* 1) Ego Position 고정 (항상 (0,0,0)) */
    egoData->Ego_Position_X = 0.0f;
    egoData->Ego_Position_Y = 0.0f;
    egoData->Ego_Position_Z = 0.0f;

    /* 2) GPS 유효성 검사 */
    float gps_dt = fabsf(timeData->Current_Time - gpsData->GPS_Timestamp);
    bool gps_valid = (gps_dt <= GPS_VALID_TIME_THRESH);

    /* 3) 센서 스파이크(노이즈) 제거 */
    float accel_x = imuData->Linear_Acceleration_X;
    float accel_y = imuData->Linear_Acceleration_Y;
    float yawRate = imuData->Yaw_Rate;
    float gps_vx  = gpsData->GPS_Velocity_X;
    float gps_vy  = gpsData->GPS_Velocity_Y;

    if(EgoEst_CheckSpike(accel_x, pState->Prev_Accel_X, MAX_SENSOR_NOISE_ACCEL))
        accel_x = pState->Prev_Accel_X;
    if(EgoEst_CheckSpike(accel_y, pState->Prev_Accel_Y, MAX_SENSOR_NOISE_ACCEL))
        accel_y = pState->Prev_Accel_Y;
    if(EgoEst_CheckSpike(yawRate, pState->Prev_YawRate, MAX_SENSOR_NOISE_YAWRATE))
        yawRate = pState->Prev_YawRate;
    if(EgoEst_CheckSpike(gps_vx, pState->Prev_GPS_VelX, MAX_SENSOR_NOISE_GPSVEL) ||
       EgoEst_CheckSpike(gps_vy, pState->Prev_GPS_VelY, MAX_SENSOR_NOISE_GPSVEL))
    {
        gps_valid = false;
    }

    pState->Prev_Accel_X = accel_x;
    pState->Prev_Accel_Y = accel_y;
    pState->Prev_YawRate   = yawRate;
    if(gps_valid) {
        pState->Prev_GPS_VelX = gps_vx;
        pState->Prev_GPS_VelY = gps_vy;
    }

    /* 4) 시간 간격 계산 (ms -> s) */
    float dt = (timeData->Current_Time - pState->Prev_Update_Time) * 0.001f;
    if(dt < 0.0f) dt = 0.01f;
    pState->Prev_Update_Time = timeData->Current_Time;

    /* 5) 예측 단계 (간단화된 칼만 필터 예시) */
    float vx = pState->X[0];
    float vy = pState->X[1];
    float ax = pState->X[2];
    float ay = pState->X[3];
    float hdg = pState->X[4];

    vx  += accel_x * dt;
    vy  += accel_y * dt;
    hdg += yawRate * dt;

    float pred_vx = vx;
    float pred_vy = vy;
    float pred_hdg = hdg;

    /* 6) 업데이트 단계 (GPS 관측) */
    if(gps_valid) {
        float obs_vx = gps_vx;
        float obs_vy = gps_vy;
        float alpha = 0.3f;
        vx = (1.0f - alpha) * pred_vx + alpha * obs_vx;
        vy = (1.0f - alpha) * pred_vy + alpha * obs_vy;
    }
    /* heading, accel는 예측값 사용 */
    hdg = pred_hdg;
    ax = accel_x;
    ay = accel_y;

    /* heading 정규화 */
    while(hdg > 180.0f)  hdg -= 360.0f;
    while(hdg < -180.0f) hdg += 360.0f;

    pState->X[0] = vx;
    pState->X[1] = vy;
    pState->X[2] = ax;
    pState->X[3] = ay;
    pState->X[4] = hdg;

    /* 7) 최종 EgoData 반영 */
    egoData->Ego_Velocity_X = vx;
    egoData->Ego_Velocity_Y = vy;
    egoData->Ego_Acceleration_X = ax;
    egoData->Ego_Acceleration_Y = ay;
    egoData->Ego_Heading = hdg;
    egoData->Ego_Yaw_Rate = yawRate;

    /* Ego_Position는 (0,0,0)로 고정 */
    egoData->Ego_Position_X = 0.0f;
    egoData->Ego_Position_Y = 0.0f;
    egoData->Ego_Position_Z = 0.0f;
}

/*============================================================================
 * 3) Lane Selection Module
 *============================================================================*/
static int LaneSelection_Compute(const LaneData_t *pLaneData,
                                 const EgoData_t  *pEgoData,
                                 LaneSelectOutput_t *pLsOut)
{
    if(!pLaneData || !pEgoData || !pLsOut) return -1;

    /* 곡선 여부 판단 */
    if(pLaneData->Lane_Curvature < 1.0f)
        pLsOut->LS_Is_Curved_Lane = false;
    else
        pLsOut->LS_Is_Curved_Lane = (pLaneData->Lane_Curvature < LANE_CURVE_THRESHOLD);

    /* 곡률 전이 플래그 */
    if(pLaneData->Next_Lane_Curvature > 0.0f) {
        float diff = fabsf(pLaneData->Next_Lane_Curvature - pLaneData->Lane_Curvature);
        pLsOut->LS_Curve_Transition_Flag = (diff > LANE_CURVE_DIFF_THRESHOLD);
    }
    else {
        pLsOut->LS_Curve_Transition_Flag = false;
    }

    pLsOut->LS_Lane_Type = pLaneData->Lane_Type;
    pLsOut->LS_Lane_Width = pLaneData->Lane_Width;
    pLsOut->LS_Lane_Offset = pLaneData->Lane_Offset;

    /* Heading Error 계산 (Ego_Heading - Lane_Heading) */
    float hdgErr = pEgoData->Ego_Heading - pLaneData->Lane_Heading;
    while(hdgErr > 180.0f)  hdgErr -= 360.0f;
    while(hdgErr < -180.0f) hdgErr += 360.0f;
    pLsOut->LS_Heading_Error = hdgErr;

    float halfWidth = pLaneData->Lane_Width * 0.5f;
    pLsOut->LS_Is_Within_Lane = (fabsf(pLaneData->Lane_Offset) < halfWidth);

    pLsOut->LS_Is_Changing_Lane = (pLaneData->Lane_Change_Status != LANE_CHANGE_KEEP);

    return 0;
}

/*============================================================================
 * 4) Target Selection Module
 *============================================================================*/
static float normalize_heading_deg(float hdg)
{
    while(hdg > 180.0f)  hdg -= 360.0f;
    while(hdg < -180.0f) hdg += 360.0f;
    return hdg;
}

/* (1) select_target_from_object_list */
static int select_target_from_object_list(const ObjectData_t *pObjList, int objCount,
                                          const EgoData_t *pEgoData,
                                          const LaneSelectOutput_t *pLsData,
                                          FilteredObject_t *pFiltList, int maxFiltCount)
{
    if(!pObjList || !pEgoData || !pLsData || !pFiltList) return 0;
    int idx = 0;
    float heading_coeff = 0.05f;
    float adjLat = pLsData->LS_Lane_Width;
    if(pLsData->LS_Is_Curved_Lane) {
        adjLat += fabsf(pLsData->LS_Heading_Error) * heading_coeff;
    }

    for(int i = 0; i < objCount; i++) {
        if(idx >= maxFiltCount) break;
        const ObjectData_t *obj = &pObjList[i];
        if(obj->Distance > MAX_OBJECT_DISTANCE) continue;

        float latPos = obj->Position_Y - pLsData->LS_Lane_Offset;
        if(fabsf(latPos) > adjLat) continue;

        /* 상태 분류 */
        float relVel = obj->Velocity_X - pEgoData->Ego_Velocity_X;
        float hdif = fabsf(obj->Heading - pEgoData->Ego_Heading);
        if(hdif > 180.0f) hdif = 360.0f - hdif;
        ObjectStatus_e stat = obj->Object_Status;
        if(hdif >= 150.0f) {
            stat = OBJSTAT_ONCOMING;
        }
        else {
            if(fabsf(relVel) >= 0.5f) stat = OBJSTAT_MOVING;
            else stat = OBJSTAT_STATIONARY;
        }

        float adjDist = obj->Distance;
        if(pLsData->LS_Is_Curved_Lane) {
            float he_rad = pLsData->LS_Heading_Error * (float)M_PI / 180.0f;
            if(fabsf(cosf(he_rad)) > 1e-3f)
                adjDist = obj->Distance / cosf(he_rad);
        }

        /* 셀 번호 부여 */
        int baseCell = 1;
        if(adjDist <= 60.0f) {
            baseCell = 1 + (int)(adjDist / 10.0f);
            if(baseCell > 6) baseCell = 6;
        }
        else if(adjDist <= 120.0f) {
            float x = adjDist - 60.0f;
            baseCell = 7 + (int)(x / 10.0f);
            if(baseCell > 12) baseCell = 12;
        }
        else {
            float x = adjDist - 120.0f;
            baseCell = 13 + (int)(x / 10.0f);
            if(baseCell > 20) baseCell = 20;
        }

        float laneOff = fabsf(obj->Position_Y - pLsData->LS_Lane_Offset);
        int offAdj = 0;
        float qW = pLsData->LS_Lane_Width * 0.25f;
        float tW = pLsData->LS_Lane_Width * 0.75f;
        if(laneOff < qW) offAdj = -1;
        else if(laneOff >= tW) offAdj = +1;
        int cellNum = baseCell + offAdj;
        if(cellNum < 1) cellNum = 1;
        if(cellNum > 20) cellNum = 20;

        FilteredObject_t *fo = &pFiltList[idx++];
        fo->Filtered_Object_ID = obj->Object_ID;
        fo->Filtered_Object_Type = obj->Object_Type;
        fo->Filtered_Position_X = obj->Position_X;
        fo->Filtered_Position_Y = obj->Position_Y;
        fo->Filtered_Position_Z = obj->Position_Z;
        fo->Filtered_Velocity_X = obj->Velocity_X;
        fo->Filtered_Velocity_Y = obj->Velocity_Y;
        fo->Filtered_Accel_X = obj->Accel_X;
        fo->Filtered_Accel_Y = obj->Accel_Y;
        fo->Filtered_Heading = normalize_heading_deg(obj->Heading);
        fo->Filtered_Distance = adjDist;
        fo->Filtered_Object_Status = stat;
        fo->Filtered_Object_Cell_ID = cellNum;
    }
    return idx;
}

/* (2) predict_object_future_path */
static int predict_object_future_path(const FilteredObject_t *pFList, int fCount,
                                      const LaneData_t *pLaneData,
                                      const LaneSelectOutput_t *pLsData,
                                      PredictedObject_t *pPList, int maxPCount)
{
    if(!pFList || !pLaneData || !pLsData || !pPList) return 0;
    int idx = 0;
    float tpred = 3.0f; // 3초 예측

    for(int i = 0; i < fCount; i++) {
        if(idx >= maxPCount) break;
        const FilteredObject_t *fo = &pFList[i];
        PredictedObject_t *po = &pPList[idx++];
        memset(po, 0, sizeof(*po));

        po->Predicted_Object_ID = fo->Filtered_Object_ID;
        po->Predicted_Object_Type = fo->Filtered_Object_Type;
        po->Predicted_Heading = fo->Filtered_Heading;
        po->Predicted_Object_Status = fo->Filtered_Object_Status;
        po->Predicted_Object_Cell_ID = fo->Filtered_Object_Cell_ID;

        float vx = fo->Filtered_Velocity_X;
        float vy = fo->Filtered_Velocity_Y;
        float ax = fo->Filtered_Accel_X;
        float ay = fo->Filtered_Accel_Y;
        float x0 = fo->Filtered_Position_X;
        float y0 = fo->Filtered_Position_Y;

        if(fo->Filtered_Object_Status == OBJSTAT_MOVING) {
            po->Predicted_Position_X = x0 + vx * tpred;
            po->Predicted_Position_Y = y0 + vy * tpred;
        }
        else {
            po->Predicted_Position_X = x0 + vx * tpred + 0.5f * ax * tpred * tpred;
            po->Predicted_Position_Y = y0 + vy * tpred + 0.5f * ay * tpred * tpred;
        }
        po->Predicted_Position_Z = fo->Filtered_Position_Z;
        po->Predicted_Velocity_X = vx;
        po->Predicted_Velocity_Y = vy;
        po->Predicted_Accel_X = ax;
        po->Predicted_Accel_Y = ay;

        float dx = po->Predicted_Position_X;
        float dy = po->Predicted_Position_Y;
        po->Predicted_Distance = sqrtf(dx * dx + dy * dy);

        po->CutIn_Flag = false;
        po->CutOut_Flag = false;
        {
            float lateralPos = po->Predicted_Position_Y - pLsData->LS_Lane_Offset;
            float cutInTh = 0.85f;
            float laneBoundary = pLsData->LS_Lane_Width * 0.5f;
            if((vx >= 0.5f) && (fabsf(vy) >= 0.2f) && (fabsf(lateralPos) <= cutInTh))
                po->CutIn_Flag = true;
            if((fabsf(vy) >= 0.2f) && (fabsf(lateralPos) > (laneBoundary + cutInTh)))
                po->CutOut_Flag = true;
        }
    }
    return idx;
}

/* (3) select_targets_for_acc_aeb */
static void select_targets_for_acc_aeb(const EgoData_t *pEgoData,
                                       const PredictedObject_t *pPList, int pCount,
                                       const LaneSelectOutput_t *pLsData,
                                       ACC_Target_t *pAccT,
                                       AEB_Target_t *pAebT)
{
    if(!pEgoData || !pPList || !pLsData || !pAccT || !pAebT) return;
    pAccT->ACC_Target_ID = -1;
    pAccT->ACC_Target_Situation = TGT_SITU_NORMAL;
    pAebT->AEB_Target_ID = -1;
    pAebT->AEB_Target_Situation = TGT_SITU_NORMAL;

    float bestAccScore = -9999.0f; int bestAccIdx = -1;
    float bestAebScore = -9999.0f; int bestAebIdx = -1;
    bool egoStopped = (fabsf(pEgoData->Ego_Velocity_X) < 0.1f);

    for(int i = 0; i < pCount; i++){
        const PredictedObject_t *obj = &pPList[i];
        if(obj->CutOut_Flag) continue;
        if(obj->Predicted_Position_X < 0.0f) continue;

        /* ACC 후보 조건 */
        if(fabsf(obj->Predicted_Position_Y) <= 1.75f &&
           obj->Predicted_Object_Type == OBJTYPE_CAR &&
           ((obj->Predicted_Object_Status == OBJSTAT_MOVING) ||
            (obj->Predicted_Object_Status == OBJSTAT_STOPPED)))
        {
            float dist = obj->Predicted_Distance;
            float score = 200.0f - dist;
            if(pLsData->LS_Is_Curved_Lane && obj->Predicted_Object_Cell_ID < 5)
                score += 10.0f;
            if(score > bestAccScore) {
                bestAccScore = score;
                bestAccIdx = i;
            }
        }

        /* AEB 후보 조건 */
        bool isFront = (fabsf(obj->Predicted_Position_Y) <= 1.75f);
        bool isSide  = (fabsf(obj->Predicted_Position_Y) > 1.75f &&
                        fabsf(obj->Predicted_Position_Y) <= 3.5f);
        bool candidate = false;
        if(isFront) {
            if(obj->Predicted_Object_Status == OBJSTAT_MOVING ||
               obj->Predicted_Object_Status == OBJSTAT_STOPPED)
                candidate = true;
            else if(obj->Predicted_Object_Status == OBJSTAT_STATIONARY && egoStopped)
                candidate = true;
        }
        else if(isSide && obj->CutIn_Flag)
            candidate = true;
        if(candidate) {
            float relSp = pEgoData->Ego_Velocity_X - obj->Predicted_Velocity_X;
            float ttc = 99999.0f;
            if(relSp > 0.1f)
                ttc = obj->Predicted_Distance / relSp;
            float score = 200.0f - obj->Predicted_Distance;
            if(obj->CutIn_Flag)
                score += 30.0f;
            if(ttc < 3.0f)
                score += 20.0f;
            if(score > bestAebScore) {
                bestAebScore = score;
                bestAebIdx = i;
            }
        }
    }

    if(bestAccIdx >= 0) {
        const PredictedObject_t *obj = &pPList[bestAccIdx];
        pAccT->ACC_Target_ID = obj->Predicted_Object_ID;
        pAccT->ACC_Target_Position_X = obj->Predicted_Position_X;
        pAccT->ACC_Target_Position_Y = obj->Predicted_Position_Y;
        pAccT->ACC_Target_Vel_X = obj->Predicted_Velocity_X;
        pAccT->ACC_Target_Vel_Y = obj->Predicted_Velocity_Y;
        pAccT->ACC_Target_Accel_X = obj->Predicted_Accel_X;
        pAccT->ACC_Target_Accel_Y = obj->Predicted_Accel_Y;
        pAccT->ACC_Target_Distance = obj->Predicted_Distance;
        pAccT->ACC_Target_Heading = obj->Predicted_Heading;
        pAccT->ACC_Target_Status = obj->Predicted_Object_Status;
        if(obj->CutIn_Flag)
            pAccT->ACC_Target_Situation = TGT_SITU_CUTIN;
        else if(obj->CutOut_Flag)
            pAccT->ACC_Target_Situation = TGT_SITU_CUTOUT;
        else if(pLsData->LS_Is_Curved_Lane)
            pAccT->ACC_Target_Situation = TGT_SITU_CURVE;
        else
            pAccT->ACC_Target_Situation = TGT_SITU_NORMAL;
    }
    if(bestAebIdx >= 0) {
        const PredictedObject_t *obj = &pPList[bestAebIdx];
        pAebT->AEB_Target_ID = obj->Predicted_Object_ID;
        pAebT->AEB_Target_Position_X = obj->Predicted_Position_X;
        pAebT->AEB_Target_Position_Y = obj->Predicted_Position_Y;
        pAebT->AEB_Target_Vel_X = obj->Predicted_Velocity_X;
        pAebT->AEB_Target_Vel_Y = obj->Predicted_Velocity_Y;
        pAebT->AEB_Target_Accel_X = obj->Predicted_Accel_X;
        pAebT->AEB_Target_Accel_Y = obj->Predicted_Accel_Y;
        pAebT->AEB_Target_Distance = obj->Predicted_Distance;
        pAebT->AEB_Target_Heading = obj->Predicted_Heading;
        pAebT->AEB_Target_Status = obj->Predicted_Object_Status;
        if(obj->CutIn_Flag)
            pAebT->AEB_Target_Situation = TGT_SITU_CUTIN;
        else if(obj->CutOut_Flag)
            pAebT->AEB_Target_Situation = TGT_SITU_CUTOUT;
        else if(pLsData->LS_Is_Curved_Lane)
            pAebT->AEB_Target_Situation = TGT_SITU_CURVE;
        else
            pAebT->AEB_Target_Situation = TGT_SITU_NORMAL;
    }
}

/*============================================================================
 * 5) ACC Module
 *============================================================================*/
static float g_distIntegral = 0.0f;
static float g_distPrevErr = 0.0f;
static float g_speedIntegral = 0.0f;
static float g_speedPrevErr = 0.0f;

static ACC_Mode_e ACC_ModeSelection(const ACC_Target_t *pAccTarget,
                                    const EgoData_t *pEgoData,
                                    const LaneSelectOutput_t *pLsData)
{
    if(!pAccTarget || !pEgoData) return ACC_MODE_SPEED;
    if(pAccTarget->ACC_Target_ID < 0) return ACC_MODE_SPEED;
    float dist = pAccTarget->ACC_Target_Distance;
    if(dist > 55.0f)
        return ACC_MODE_SPEED;
    else if(dist < 45.0f)
        return ACC_MODE_DISTANCE;
    else {
        if(pAccTarget->ACC_Target_Status == OBJSTAT_STOPPED &&
           pEgoData->Ego_Velocity_X < 0.5f)
            return ACC_MODE_STOP;
        return ACC_MODE_SPEED;
    }
}

static float ACC_CalcAccel_Distance(const ACC_Target_t *pAccTarget,
                                    const EgoData_t *pEgoData,
                                    float dt)
{
    if(!pAccTarget || !pEgoData) return 0.0f;
    float targetDist = 40.0f;
    float distErr = targetDist - pAccTarget->ACC_Target_Distance;
    float Kp = 0.4f, Ki = 0.05f, Kd = 0.1f;
    g_distIntegral += distErr * dt;
    float dErr = (distErr - g_distPrevErr) / (dt + 1e-5f);
    g_distPrevErr = distErr;
    float accelDist = (Kp * distErr) + (Ki * g_distIntegral) + (Kd * dErr);
    if(pAccTarget->ACC_Target_Status == OBJSTAT_STOPPED && pEgoData->Ego_Velocity_X < 0.5f)
        accelDist = -3.0f;
    return accelDist;
}

static float ACC_CalcAccel_Speed(const EgoData_t *pEgoData,
                                 const LaneSelectOutput_t *pLsData,
                                 float dt)
{
    if(!pEgoData || !pLsData) return 0.0f;
    float baseSpeed = 22.22f; /* 80km/h = 22.22 m/s */
    if(pLsData->LS_Is_Curved_Lane && baseSpeed > 15.0f)
        baseSpeed = 15.0f;
    float speedErr = baseSpeed - pEgoData->Ego_Velocity_X;
    float Kp = 0.5f, Ki = 0.1f, Kd = 0.05f;
    g_speedIntegral += speedErr * dt;
    float dErr = (speedErr - g_speedPrevErr) / (dt + 1e-5f);
    g_speedPrevErr = speedErr;
    float accelSpeed = (Kp * speedErr) + (Ki * g_speedIntegral) + (Kd * dErr);
    return accelSpeed;
}

static float ACC_OutputSelection(ACC_Mode_e mode, float accelDist, float accelSpeed)
{
    if(mode == ACC_MODE_SPEED)
        return accelSpeed;
    else if(mode == ACC_MODE_DISTANCE)
        return accelDist;
    else if(mode == ACC_MODE_STOP)
        return 0.0f;
    return 0.0f;
}

/*============================================================================
 * 6) AEB Module
 *============================================================================*/
static void AEB_CalcTTC(const EgoData_t *pEgo,
                        const AEB_Target_t *pAeb,
                        float *pTTC,
                        float *pTTC_Brake,
                        float *pTTC_Alert,
                        float *pRelSp)
{
    if(!pEgo || !pAeb || !pTTC || !pTTC_Brake || !pTTC_Alert || !pRelSp)
        return;
    *pTTC = 99999.0f;
    *pTTC_Brake = 0.0f;
    *pTTC_Alert = 0.0f;
    *pRelSp = 0.0f;
    if(pAeb->AEB_Target_ID < 0) return;
    float relSpd = pEgo->Ego_Velocity_X - pAeb->AEB_Target_Vel_X;
    if(relSpd <= 0.0f) return;
    *pRelSp = relSpd;
    float dist = pAeb->AEB_Target_Distance;
    if(dist < 0.01f) dist = 0.01f;
    float ttc = dist / relSpd;
    *pTTC = ttc;
    float maxDecel = AEB_DEFAULT_MAX_DECEL;
    float tbrake = 0.0f;
    if(pEgo->Ego_Velocity_X > 0.1f)
        tbrake = pEgo->Ego_Velocity_X / maxDecel;
    *pTTC_Brake = tbrake;
    *pTTC_Alert = tbrake + AEB_ALERT_BUFFER_TIME;
}

static AEB_Mode_e AEB_ModeSelection(float ttc, float ttcBrake, float ttcAlert,
                                    const AEB_Target_t *pAeb,
                                    const EgoData_t *pEgo)
{
    if(!pAeb || !pEgo) return AEB_MODE_NORMAL;
    if(pAeb->AEB_Target_ID < 0) return AEB_MODE_NORMAL;
    if(pEgo->Ego_Velocity_X < 0.5f) return AEB_MODE_NORMAL;
    if(ttc <= 0.0f || ttc >= 99999.0f) return AEB_MODE_NORMAL;
    if(pAeb->AEB_Target_Situation == TGT_SITU_CUTOUT) return AEB_MODE_NORMAL;
    if(ttc > ttcAlert)
        return AEB_MODE_NORMAL;
    else if(ttc > ttcBrake && ttc <= ttcAlert)
        return AEB_MODE_ALERT;
    else if(ttc > 0.0f && ttc <= ttcBrake)
        return AEB_MODE_BRAKE;
    return AEB_MODE_NORMAL;
}

static float AEB_CalcDecel(AEB_Mode_e mode, float ttc, float ttcBrake)
{
    if(mode == AEB_MODE_NORMAL || mode == AEB_MODE_ALERT)
        return 0.0f;
    else if(mode == AEB_MODE_BRAKE) {
        float ratio = 1.0f - (ttc / (ttcBrake + 1e-5f));
        float dec = AEB_MAX_BRAKE_DECEL * ratio;
        if(dec > AEB_MIN_BRAKE_DECEL)
            dec = AEB_MIN_BRAKE_DECEL;
        if(dec < AEB_MAX_BRAKE_DECEL)
            dec = AEB_MAX_BRAKE_DECEL;
        return dec;
    }
    return 0.0f;
}

/*============================================================================
 * 7) LFA Module
 *============================================================================*/
static float g_lfaPidIntegral = 0.0f;
static float g_lfaPidPrevErr = 0.0f;

static LFA_Mode_e LFA_ModeSelection(const EgoData_t *pEgo)
{
    if(!pEgo) return LFA_MODE_LOW_SPEED;
    if(pEgo->Ego_Velocity_X < LFA_LOW_SPEED_THRESHOLD)
        return LFA_MODE_LOW_SPEED;
    else
        return LFA_MODE_HIGH_SPEED;
}

static float LFA_CalcSteer_LowSpeedPID(const LaneSelectOutput_t *pLs, float dt)
{
    if(!pLs) return 0.0f;
    float offsetErr = pLs->LS_Lane_Offset;
    float headingErr = pLs->LS_Heading_Error;
    float K_offset = 1.0f, K_heading = 1.0f;
    float error = K_offset * offsetErr + K_heading * headingErr;
    float Kp = 0.1f, Ki = 0.01f, Kd = 0.005f;
    g_lfaPidIntegral += error * dt;
    float dErr = (error - g_lfaPidPrevErr) / (dt + 1e-5f);
    g_lfaPidPrevErr = error;
    float steer = (Kp * error) + (Ki * g_lfaPidIntegral) + (Kd * dErr);
    if(steer > LFA_MAX_STEERING_ANGLE) steer = LFA_MAX_STEERING_ANGLE;
    if(steer < -LFA_MAX_STEERING_ANGLE) steer = -LFA_MAX_STEERING_ANGLE;
    return steer;
}

static float LFA_CalcSteer_HighSpeedStanley(const EgoData_t *pEgo,
                                            const LaneSelectOutput_t *pLs)
{
    if(!pEgo || !pLs) return 0.0f;
    float vx = pEgo->Ego_Velocity_X;
    if(vx < 0.1f) vx = 0.1f;
    float stGain = 1.0f;
    float offset = pLs->LS_Lane_Offset;
    float hdgErr = pLs->LS_Heading_Error;
    float angleOffsetRad = atanf((stGain * offset) / vx);
    float angleOffsetDeg = angleOffsetRad * 180.0f / M_PI;
    float steer = hdgErr + angleOffsetDeg;
    if(steer > LFA_MAX_STEERING_ANGLE) steer = LFA_MAX_STEERING_ANGLE;
    if(steer < -LFA_MAX_STEERING_ANGLE) steer = -LFA_MAX_STEERING_ANGLE;
    return steer;
}

static float LFA_OutputSelection(LFA_Mode_e mode,
                                 float steerPid,
                                 float steerStanley,
                                 const LaneSelectOutput_t *pLs,
                                 const EgoData_t *pEgo)
{
    float steering = 0.0f;
    if(mode == LFA_MODE_LOW_SPEED)
        steering = steerPid;
    else
        steering = steerStanley;
    if(pLs->LS_Is_Changing_Lane)
        steering *= 0.2f;
    if(!pLs->LS_Is_Within_Lane)
        steering *= 1.5f;
    float curveGain = 1.0f;
    if(pLs->LS_Is_Curved_Lane) {
        curveGain = 1.2f;
        if(pEgo) {
            float yawThr = 30.0f;
            float steerThr = 200.0f;
            if(fabsf(pEgo->Ego_Yaw_Rate) > yawThr || fabsf(pEgo->Ego_Heading) > steerThr)
                curveGain = 0.8f;
        }
    }
    steering *= curveGain;
    if(steering > LFA_MAX_STEERING_ANGLE) steering = LFA_MAX_STEERING_ANGLE;
    if(steering < -LFA_MAX_STEERING_ANGLE) steering = -LFA_MAX_STEERING_ANGLE;
    return steering;
}

/*============================================================================
 * 8) Arbitration Module
 *============================================================================*/
typedef struct {
    float throttle; /* 0.0 ~ 1.0 */
    float brake;    /* 0.0 ~ 1.0 */
    float steer;    /* -1.0 ~ 1.0 */
} VehicleControl_t;

static void Arbitration_ComputeControl(float accelAccX,
                                       float decelAebX,
                                       float steerLfa,
                                       AEB_Mode_e aebMode,
                                       VehicleControl_t *pOut)
{
    if(!pOut) return;
    float selectedAccel = 0.0f;
    float MAX_THROTTLE_ACCEL = 10.0f;
    float MAX_BRAKE_DECEL = -10.0f;
    float MAX_STEER_ANGLE = 540.0f;

    if(aebMode == AEB_MODE_BRAKE)
        selectedAccel = decelAebX;
    else if(aebMode == AEB_MODE_NORMAL || aebMode == AEB_MODE_ALERT)
        selectedAccel = accelAccX;

    float throttle = 0.0f, brake = 0.0f;
    if(selectedAccel > 0.0f) {
        float ratio = selectedAccel / MAX_THROTTLE_ACCEL;
        if(ratio > 1.0f) ratio = 1.0f;
        if(ratio < 0.0f) ratio = 0.0f;
        throttle = ratio;
        brake = 0.0f;
    }
    else if(selectedAccel < 0.0f) {
        float ratio = fabsf(selectedAccel / MAX_BRAKE_DECEL);
        if(ratio > 1.0f) ratio = 1.0f;
        brake = ratio;
        throttle = 0.0f;
    }
    else {
        throttle = 0.0f;
        brake = 0.0f;
    }

    float steerRatio = steerLfa / MAX_STEER_ANGLE;
    if(steerRatio > 1.0f) steerRatio = 1.0f;
    if(steerRatio < -1.0f) steerRatio = -1.0f;
    pOut->throttle = throttle;
    pOut->brake = brake;
    pOut->steer = steerRatio;
}

/*============================================================================
 * 9) main() : 전체 흐름 데모
 *============================================================================*/
int main(void)
{
    /* 가상 입력 생성 */
    TimeData_t timeData = { .Current_Time = 10.0f };  // ms
    GPSData_t gpsData = { .GPS_Velocity_X = 10.0f, .GPS_Velocity_Y = 0.0f, .GPS_Timestamp = 10.0f };
    IMUData_t imuData = { .Linear_Acceleration_X = 0.0f, .Linear_Acceleration_Y = 0.0f, .Yaw_Rate = 0.0f };
    EgoData_t egoData;
    memset(&egoData, 0, sizeof(egoData));
    
    EgoEstimationKFState_t egoEstState;
    EgoEstimation_Init(&egoEstState);

    /* Lane Data */
    LaneData_t laneData = {
        .Lane_Type = LANE_TYPE_STRAIGHT,
        .Lane_Curvature = 0.0f,
        .Next_Lane_Curvature = 0.0f,
        .Lane_Offset = 0.0f,
        .Lane_Heading = 0.0f,
        .Lane_Width = 3.5f,
        .Lane_Change_Status = LANE_CHANGE_KEEP
    };
    LaneSelectOutput_t laneSelOut;
    memset(&laneSelOut, 0, sizeof(laneSelOut));

    /* Object list */
    ObjectData_t objList[3] = {
        { .Object_ID = 1, .Object_Type = OBJTYPE_CAR, .Position_X = 30.0f, .Position_Y = 0.3f, 
          .Distance = 30.0f, .Velocity_X = 8.0f, .Heading = 0.0f, .Object_Status = OBJSTAT_MOVING },
        { .Object_ID = 2, .Object_Type = OBJTYPE_PEDESTRIAN, .Position_X = 25.0f, .Position_Y = 2.2f,
          .Distance = 25.0f, .Velocity_X = 1.0f, .Heading = 10.0f, .Object_Status = OBJSTAT_MOVING },
        { .Object_ID = 3, .Object_Type = OBJTYPE_CAR, .Position_X = 120.0f, .Position_Y = -0.4f,
          .Distance = 120.0f, .Velocity_X = 12.0f, .Heading = 0.0f, .Object_Status = OBJSTAT_MOVING }
    };
    FilteredObject_t fList[3];
    PredictedObject_t pList[3];
    memset(fList, 0, sizeof(fList));
    memset(pList, 0, sizeof(pList));

    ACC_Target_t accTarget;
    memset(&accTarget, 0, sizeof(accTarget));
    AEB_Target_t aebTarget;
    memset(&aebTarget, 0, sizeof(aebTarget));

    /* 1) Ego Vehicle Estimation */
    EgoEstimation_Update(&timeData, &gpsData, &imuData, &egoData, &egoEstState);

    /* 2) Lane Selection */
    LaneSelection_Compute(&laneData, &egoData, &laneSelOut);

    /* 3) Target Selection */
    int filtCount = select_target_from_object_list(objList, 3, &egoData, &laneSelOut, fList, 3);
    int predCount = predict_object_future_path(fList, filtCount, &laneData, &laneSelOut, pList, 3);
    select_targets_for_acc_aeb(&egoData, pList, predCount, &laneSelOut, &accTarget, &aebTarget);

    /* 4) ACC 계산 */
    float dt = 0.01f; // 10ms
    ACC_Mode_e accMode = ACC_ModeSelection(&accTarget, &egoData, &laneSelOut);
    float accelDist = ACC_CalcAccel_Distance(&accTarget, &egoData, dt);
    float accelSpeed = ACC_CalcAccel_Speed(&egoData, &laneSelOut, dt);
    float accelACC = ACC_OutputSelection(accMode, accelDist, accelSpeed);

    /* 5) AEB 계산 */
    float ttc = 0.0f, ttcBrake = 0.0f, ttcAlert = 0.0f, relSpd = 0.0f;
    AEB_CalcTTC(&egoData, &aebTarget, &ttc, &ttcBrake, &ttcAlert, &relSpd);
    AEB_Mode_e aebMode = AEB_ModeSelection(ttc, ttcBrake, ttcAlert, &aebTarget, &egoData);
    float decelAEB = AEB_CalcDecel(aebMode, ttc, ttcBrake);

    /* 6) LFA 계산 */
    LFA_Mode_e lfaMode = LFA_ModeSelection(&egoData);
    float steerPid = LFA_CalcSteer_LowSpeedPID(&laneSelOut, dt);
    float steerStanley = LFA_CalcSteer_HighSpeedStanley(&egoData, &laneSelOut);
    float steerLFA = LFA_OutputSelection(lfaMode, steerPid, steerStanley, &laneSelOut, &egoData);

    /* 7) Arbitration */
    VehicleControl_t ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    Arbitration_ComputeControl(accelACC, decelAEB, steerLFA, aebMode, &ctrl);

    /* 결과 출력 */
    printf("=== adas_all_in_one : Demo ===\n");
    printf("Ego: VelX=%.2f, Heading=%.2f\n", egoData.Ego_Velocity_X, egoData.Ego_Heading);
    printf("ACC Mode=%d, Accel_ACC=%.2f\n", (int)accMode, accelACC);
    printf("AEB Mode=%d, Decel_AEB=%.2f, TTC=%.2f\n", (int)aebMode, decelAEB, ttc);
    printf("LFA Mode=%d, Steer_LFA=%.2f deg\n", (int)lfaMode, steerLFA);
    printf("--- Arbitration Final ---\n");
    printf("Throttle=%.2f, Brake=%.2f, Steer=%.2f\n", ctrl.throttle, ctrl.brake, ctrl.steer);

    return 0;
}
