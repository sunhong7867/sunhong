/****************************************************************************
 * adas_shared.h
 *
 * - 전 모듈(Ego, Lane, Target, ACC, AEB, LFA, Arbitration)에서 공통 쓰이는 구조체 & 상수
 * - "Cut-in / Cut-out" & "곡선 여부" 등 반영
 ****************************************************************************/
#ifndef ADAS_SHARED_H
#define ADAS_SHARED_H

#include <stdbool.h>

/* M_PI가 math.h에 없을 경우 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================
 * 1) Time, GPS, IMU, EgoData
 *============================================================*/
typedef struct {
    float Current_Time;  /* [ms], 제어루프 시각 */
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

/* Ego 차량 정보 */
typedef struct {
    float Ego_Velocity_X;     /* [m/s] */
    float Ego_Velocity_Y;     /* [m/s] */
    float Ego_Acceleration_X; /* [m/s^2] */
    float Ego_Acceleration_Y; /* [m/s^2] */
    float Ego_Heading;        /* [deg], -180~180 */
    float Ego_Yaw_Rate;       /* [deg/s] */

    float Ego_Position_X; 
    float Ego_Position_Y; 
    float Ego_Position_Z; 
} EgoData_t;

/*=============================================================
 * 2) Lane Data & Output
 *============================================================*/
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

/*=============================================================
 * 3) Target Selection 자료형
 *============================================================*/
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

/* Raw/Fused Object */
typedef struct {
    int   Object_ID;
    ObjectType_e  Object_Type;
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

/* Filtered Object */
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

/* Predicted Object (Cut-in/out 판단) */
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
#define LATERAL_THRESHOLD    4.0f   /* ±4m */

/* 타겟 상황 (Normal, Cut-in, Cut-out, Curve 등) */
typedef enum {
    TGT_SITU_NORMAL = 0,
    TGT_SITU_CUTIN,
    TGT_SITU_CUTOUT,
    TGT_SITU_CURVE
} TargetSituation_e;

/*=============================================================
 * 4) ACC/AEB Target
 *============================================================*/
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
    ObjectStatus_e   ACC_Target_Status;    /* Moving,Stopped,Stationary,Oncoming */
    TargetSituation_e ACC_Target_Situation;/* Normal,CutIn,CutOut,Curve */
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
    ObjectStatus_e   AEB_Target_Status;   /* Moving,Stopped,Stationary,Oncoming */
    TargetSituation_e AEB_Target_Situation;/* Normal,CutIn,CutOut,Curve */
} AEB_Target_t;

/*=============================================================
 * 5) ACC/AEB 모드
 *============================================================*/
/* 설계 코드에서 ACC는 'ACC_Mode_e'라 명명하므로 통일 */
typedef enum {
    ACC_MODE_SPEED = 0,
    ACC_MODE_DISTANCE,
    ACC_MODE_STOP
} ACC_Mode_e;

#define MAX_ACCEL  10.0f
#define MIN_ACCEL -10.0f

/* AEB 모드 */
typedef enum {
    AEB_MODE_NORMAL = 0,
    AEB_MODE_ALERT,
    AEB_MODE_BRAKE
} AEB_Mode_e;

/* AEB 상수들 */
#define AEB_MAX_BRAKE_DECEL  -10.0f
#define AEB_MIN_BRAKE_DECEL  -2.0f
#define AEB_MAX_BRAKE_SPEED   0.5f
#define AEB_ALERT_BUFFER_TIME 1.2f
#define AEB_DEFAULT_MAX_DECEL 9.0f

/*=============================================================
 * 6) LFA 모드
 *============================================================*/
typedef enum {
    LFA_MODE_LOW_SPEED = 0,
    LFA_MODE_HIGH_SPEED
} LFA_Mode_e;

#define LFA_LOW_SPEED_THRESHOLD 16.67f
#define LFA_MAX_STEERING_ANGLE  540.0f

#ifdef __cplusplus
}
#endif

#endif /* ADAS_SHARED_H */
