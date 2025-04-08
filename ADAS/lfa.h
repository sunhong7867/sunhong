#ifndef LFA_H
#define LFA_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LFA 모드 (저속/고속)
 */
typedef enum
{
    LFA_MODE_LOW_SPEED = 0,
    LFA_MODE_HIGH_SPEED
} LFA_Mode_e;

/**
 * @brief Lane Data(LS)
 *  - LS_Heading_Error: 진행 방향과 차선 방향 차이 (°)
 *  - LS_Lane_Offset: 차선 중심 대비 오프셋 (m)
 *  - LS_Is_Changing_Lane: 차선 변경 여부
 *  - LS_Is_Within_Lane: 차선 내 여부
 *  - LS_Is_Curved_Lane: 곡선 차선 여부
 */
typedef struct
{
    float LS_Heading_Error;    /* (-180 ~ 180) [°] */
    float LS_Lane_Offset;      /* (-2.0, 2.0) [m] */
    int   LS_Is_Changing_Lane; /* (True=1, False=0) */
    int   LS_Is_Within_Lane;   /* (True=1, False=0) */
    int   LS_Is_Curved_Lane;   /* (True=1, False=0) */
} Lane_Data_LS_t;

/**
 * @brief Ego 차량 정보
 *  - Ego_Velocity_X: 종방향 속도 [m/s]
 *  - Ego_Yaw_Rate: Yaw 회전 속도 [°/s]
 *  - Ego_Steering_Angle: 현재 조향각 [°]
 */
typedef struct
{
    float Ego_Velocity_X;      /* (0, 100) [m/s] */
    float Ego_Yaw_Rate;        /* (-180, 180) [°/s] */
    float Ego_Steering_Angle;  /* (-540, 540) [°] */
} Ego_Data_t;

/**
 * @brief LFA 모드 선택 함수 (2.2.4.1.1)
 * @param pEgoData  : (입력) Ego 차량 속도
 * @return LFA_Mode_e: (출력) LOW_SPEED or HIGH_SPEED
 */
LFA_Mode_e lfa_mode_selection(const Ego_Data_t *pEgoData);

/**
 * @brief 저속 모드 PID 조향각 계산 (2.2.4.1.2)
 * @param pLaneData : (입력) 차선 오차, heading 오차
 * @param deltaTime : (입력) 제어 루프 시간 간격
 * @return float     : Steering_Angle_PID (-540 ~ 540) [°]
 */
float calculate_steer_in_low_speed_pid(const Lane_Data_LS_t *pLaneData,
                                       float deltaTime);

/**
 * @brief 고속 모드 Stanley 조향각 계산 (2.2.4.1.3)
 * @param pEgoData  : (입력) Ego 차량 속도
 * @param pLaneData : (입력) 차선 오차, heading 오차
 * @return float     : Steering_Angle_Stanley (-540 ~ 540) [°]
 */
float calculate_steer_in_high_speed_stanley(const Ego_Data_t    *pEgoData,
                                            const Lane_Data_LS_t *pLaneData);

/**
 * @brief LFA 최종 출력 선택 (2.2.4.1.4)
 * @param lfaMode        : (입력) 현재 모드(LOW_SPEED/HIGH_SPEED)
 * @param steeringAnglePID      : (입력) PID 조향각
 * @param steeringAngleStanley  : (입력) Stanley 조향각
 * @param pLaneData      : (입력) 차선 변경/이탈/곡선 여부 등
 * @param pEgoData       : (입력) 차량의 YawRate, SteeringAngle 등
 * @return float         : 최종 조향각 Steer_LFA_Angle (-540 ~ 540)
 */
float lfa_output_selection(LFA_Mode_e lfaMode,
                           float steeringAnglePID,
                           float steeringAngleStanley,
                           const Lane_Data_LS_t *pLaneData,
                           const Ego_Data_t     *pEgoData);

#ifdef __cplusplus
}
#endif

#endif /* LFA_H */
