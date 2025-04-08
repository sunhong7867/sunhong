#include <stdio.h>
#include "adas_shared.h"
#include "ego_estimation.h"
#include "lane_selection.h"
#include "target_selection.h"
#include "acc.h"
#include "aeb.h"
#include "lfa.h"
#include "arbitration.h"

int main(void)
{
    /* 가상의 입력 */
    TimeData_t timeData = { .Current_Time=0.0f };
    GPSData_t  gpsData  = { .GPS_Velocity_X=10.0f, .GPS_Velocity_Y=0.0f, .GPS_Timestamp=0.0f };
    IMUData_t  imuData  = { .Linear_Acceleration_X=0.0f, .Linear_Acceleration_Y=0.0f, .Yaw_Rate=0.0f };
    EgoData_t  egoData;
    EgoEstimationKFState_t kfState;
    EgoEstimation_Init(&kfState);

    LaneData_t laneData = {
        .Lane_Type=LANE_TYPE_STRAIGHT,
        .Lane_Curvature=0.0f,
        .Next_Lane_Curvature=0.0f,
        .Lane_Offset=0.0f,
        .Lane_Heading=0.0f,
        .Lane_Width=3.5f,
        .Lane_Change_Status=LANE_CHANGE_KEEP
    };
    LaneSelectOutput_t laneSelOut;

    /* object list */
    ObjectData_t objList[3] = {
        { .Object_ID=1, .Object_Type=OBJTYPE_CAR, .Position_X=30.0f, .Position_Y=0.5f, 
          .Distance=30.0f, .Velocity_X=8.0f, .Heading=0.0f, .Object_Status=OBJSTAT_MOVING },
        { .Object_ID=2, .Object_Type=OBJTYPE_PEDESTRIAN, .Position_X=25.0f, .Position_Y=2.0f,
          .Distance=26.0f, .Velocity_X=1.0f, .Heading=10.0f, .Object_Status=OBJSTAT_MOVING },
        { .Object_ID=3, .Object_Type=OBJTYPE_CAR, .Position_X=100.0f, .Position_Y=-0.5f,
          .Distance=100.0f, .Velocity_X=12.0f, .Heading=0.0f, .Object_Status=OBJSTAT_MOVING }
    };
    FilteredObject_t fList[3];
    PredictedObject_t pList[3];

    ACC_Target_t accTarget;
    AEB_Target_t aebTarget;

    /* 모의 루프 한번 */
    /* 1) EgoEstimation_Update */
    timeData.Current_Time= 10.0f; /* ms */
    EgoEstimation_Update(&timeData, &gpsData, &imuData, &egoData, &kfState);

    /* 2) LaneSelection */
    LaneSelection_Compute(&laneData, &egoData, &laneSelOut);

    /* 3) TargetSelection */
    int fcount = select_target_from_object_list(objList, 3, &egoData, &laneSelOut,
                                                fList, 3);
    int pcount = predict_object_future_path(fList, fcount, &laneData, &laneSelOut,
                                            pList, 3);
    select_targets_for_acc_aeb(&egoData, pList, pcount, &laneSelOut,
                               &accTarget, &aebTarget);

    /* 4) ACC */
    ACCMode_e accMode = ACC_ModeSelection(&accTarget, &egoData, &laneSelOut);
    float dt=0.01f; /* 10ms */
    float accDist= ACC_CalcAccel_Distance(&accTarget, &egoData, dt);
    float accSpeed=ACC_CalcAccel_Speed(&egoData, &laneSelOut, dt);
    float accelACC= ACC_OutputSelection(accMode, accDist, accSpeed);

    /* 5) AEB */
    float ttc=0.0f, ttcBrake=0.0f, ttcAlert=0.0f, relSp=0.0f;
    AEB_CalcTTC(&egoData, &aebTarget, &ttc, &ttcBrake, &ttcAlert, &relSp);
    AEB_Mode_e aebMode = AEB_ModeSelection(ttc, ttcBrake, ttcAlert, &aebTarget, &egoData);
    float decelAEB= AEB_CalcDecel(aebMode, ttc, ttcBrake);

    /* 6) LFA */
    LFA_Mode_e lfaMode= LFA_ModeSelection(&egoData);
    float steerPid= LFA_CalcSteer_LowSpeedPID(&laneSelOut, dt);
    float steerStan= LFA_CalcSteer_HighSpeedStanley(&egoData, &laneSelOut);
    float steerLFA= LFA_OutputSelection(lfaMode, steerPid, steerStan, &laneSelOut, &egoData);

    /* 7) Arbitration */
    VehicleControl_t ctrl;
    Arbitration_ComputeControl(accelACC, decelAEB, steerLFA, aebMode, &ctrl);

    printf("---- EgoData ----\n");
    printf("VelX=%.2f, Heading=%.2f\n", egoData.Ego_Velocity_X, egoData.Ego_Heading);

    printf("---- ACC Output ----\n");
    printf("ACC Mode=%d, ACC Accel=%.2f\n",(int)accMode, accelACC);

    printf("---- AEB Output ----\n");
    printf("AEB Mode=%d, Decel=%.2f, TTC=%.2f\n",(int)aebMode,decelAEB,ttc);

    printf("---- LFA Output ----\n");
    printf("LFA Mode=%d, Steer=%.2f deg\n",(int)lfaMode, steerLFA);

    printf("---- Arbitration Final ----\n");
    printf("Throttle=%.2f, Brake=%.2f, Steer=%.2f\n", ctrl.throttle, ctrl.brake, ctrl.steer);

    return 0;
}
