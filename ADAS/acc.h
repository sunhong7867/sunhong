#ifndef ACC_H
#define ACC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ACC 제어 모드
 * (Speed, Distance, Stop)
 */
typedef enum
{
    ACC_MODE_SPEED = 0,
    ACC_MODE_DISTANCE,
    ACC_MODE_STOP
} ACC_Mode_e;

/**
 * @brief ACC 타겟 상태 (Moving, Stopped, Stationary, Oncoming)
 */
typedef enum
{
    ACC_TARGET_MOVING = 0,
    ACC_TARGET_STOPPED,
    ACC_TARGET_STATIONARY,
    ACC_TARGET_ONCOMING
} ACC_Target_Status_e;

/**
 * @brief ACC 타겟 상황 (Normal, Cut-in, Cut-out)
 */
typedef enum
{
    ACC_TARGET_NORMAL = 0,
    ACC_TARGET_CUT_IN,
    ACC_TARGET_CUT_OUT
} ACC_Target_Situation_e;

/**
 * @brief ACC_Target Data 구조체
 * 설계서: 2.2.4.1.1 ~ 2.2.4.1.2
 */
typedef struct
{
    int   ACC_Target_ID;           /* (0, N)  */
    float ACC_Target_Distance;     /* (0, 200) [m] */
    ACC_Target_Status_e    ACC_Target_Status;    /* (Moving, Stopped, Stationary, Oncoming) */
    ACC_Target_Situation_e ACC_Target_Situation; /* (Normal, Cut-in, Cut-out) */

    float ACC_Target_Velocity_X;   /* (0, 100) [m/s] */
    /* 필요하다면 횡방향 속도, 가속도 등 추가 */
} ACC_Target_Data_t;

/**
 * @brief Ego 차량 정보 (Ego Data)
 * 설계서: 2.2.4.1.1, 2.2.4.1.2 ~ 2.2.4.1.3
 */
typedef struct
{
    float Ego_Velocity_X;       /* (0, 100) [m/s] */
    float Ego_Acceleration_X;   /* (-10, 10) [m/s²] */
    /* 필요하다면 Ego_Velocity_Y, Heading 등 추가 */
} Ego_Data_t;

/**
 * @brief 차선 데이터 (Lane Data)
 * 설계서: 2.2.4.1.1, 2.2.4.1.3
 * waypoint(곡률) + LS(Heading Error, Is_Curved_Lane) 함께 사용
 */
typedef struct
{
    float Lane_Curvature;       /* (0 ~ ∞) [m] */
    float Next_Lane_Curvature;  /* (0 ~ ∞) [m] */
    float LS_Heading_Error;     /* (-180 ~ 180) [°] */
    int   LS_Is_Curved_Lane;    /* (True=1, False=0) */
} Lane_Data_t;

/**
 * @brief 2.2.4.1.1 acc_mode_selection
 * ACC 제어 모드(Speed, Distance, Stop) 결정
 */
ACC_Mode_e acc_mode_selection(
    const ACC_Target_Data_t *pAccTargetData,
    const Ego_Data_t        *pEgoData,
    const Lane_Data_t       *pLaneData
);

/**
 * @brief 2.2.4.1.2 calculate_accel_for_distance_pid
 * 거리 모드에서의 종방향 가속도 계산
 */
float calculate_accel_for_distance_pid(
    ACC_Mode_e               accMode,          /* (Speed, Distance, Stop) */
    const ACC_Target_Data_t *pAccTargetData,
    const Ego_Data_t        *pEgoData,
    float                    current_time
);

/**
 * @brief 2.2.4.1.3 calculate_accel_for_speed_pid
 * 속도 모드에서의 종방향 가속도 계산
 */
float calculate_accel_for_speed_pid(
    const Ego_Data_t  *pEgoData,
    const Lane_Data_t *pLaneData,
    float              delta_time
);

/**
 * @brief 2.2.4.1.4 acc_output_selection
 * 두 PID 결과(거리/속도) 중 최종 출력 가속도를 선택
 */
float acc_output_selection(
    ACC_Mode_e accMode,
    float      Accel_Distance_X,
    float      Accel_Speed_X
);

#ifdef __cplusplus
}
#endif

#endif /* ACC_H */
