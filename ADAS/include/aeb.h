#ifndef AEB_H
#define AEB_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AEB 모드 (Normal, Alert, Brake)
 */
typedef enum
{
    AEB_MODE_NORMAL = 0,
    AEB_MODE_ALERT,
    AEB_MODE_BRAKE
} AEB_Mode_e;

/**
 * @brief AEB 타겟 상황 (Normal, Cut-in, Cut-out)
 */
typedef enum
{
    AEB_TARGET_NORMAL = 0,
    AEB_TARGET_CUT_IN,
    AEB_TARGET_CUT_OUT
} AEB_Target_Situation_e;

/**
 * @brief AEB_Target Data 구조체
 * (설계서 2.2.3.1 Input Data)
 */
typedef struct
{
    int   AEB_Target_ID;         /* (0, N)   */
    float AEB_Target_Distance;   /* (0, 200) [m]  */
    float AEB_Target_Velocity_X; /* (0, 100) [m/s] */
    AEB_Target_Situation_e AEB_Target_Situation; /* (Normal, Cut-in, Cut-out) */
} AEB_Target_Data_t;

/**
 * @brief Ego 차량 정보 (Ego Data)
 * (설계서 2.2.3.1 Input Data)
 */
typedef struct
{
    float Ego_Velocity_X;   /* (0, 100) [m/s] */
    /* 필요시 Ego_Acceleration_X 등 추가 가능 */
} Ego_Data_t;

/**
 * @brief TTC Data 구조체
 * (설계서 Output Data 예시에 따라)
 */
typedef struct
{
    float TTC;          /* (0, ∞) [s]  */
    float TTC_Brake;    /* (0, 10) [s] */
    float TTC_Alert;    /* (0, 10) [s] */
    float Relative_Speed; /* (-100, 100) [m/s] */
} TTC_Data_t;

/* 최대 감속 성능, 경고 시 여유시간 등 상수 정의 */
#define AEB_MAX_BRAKE_DECEL         (-10.0f)  /* 실제 제동시 최대 감속도 [m/s^2] */
#define AEB_MIN_BRAKE_DECEL         (-2.0f)   /* 최소 감속도 (약하게 브레이크) [m/s^2] */
#define AEB_DEFAULT_MAX_DECEL       (9.0f)    /* TTC_Brake 계산용(양수) */
#define AEB_ALERT_BUFFER_TIME       (1.2f)    /* 경고 여유시간 */

/**
 * @brief 2.2.3.1.1 calculate_ttc_for_aeb
 * Ego 차량과 AEB 타겟 간의 상대 속도 및 거리 정보를 기반으로 TTC를 계산.
 * @param pAebTargetData  (입력) AEB 타겟 정보
 * @param pEgoData        (입력) Ego 차량 정보
 * @param pTtcData        (출력) TTC, TTC_Brake, TTC_Alert, Relative_Speed
 */
void calculate_ttc_for_aeb(const AEB_Target_Data_t *pAebTargetData,
                           const Ego_Data_t        *pEgoData,
                           TTC_Data_t              *pTtcData);

/**
 * @brief 2.2.3.1.2 aeb_mode_selection
 * TTC 관련 신호를 바탕으로, Ego 차량이 충돌 위험 상황을 판단하고 AEB 모드를 결정.
 * @param pAebTargetData (입력)  AEB 타겟 정보
 * @param pEgoData       (입력)  Ego 차량 정보
 * @param pTtcData       (입력)  계산된 TTC Data
 * @return AEB_Mode_e    (출력)  현재 AEB 모드 (Normal, Alert, Brake)
 */
AEB_Mode_e aeb_mode_selection(const AEB_Target_Data_t *pAebTargetData,
                              const Ego_Data_t        *pEgoData,
                              const TTC_Data_t        *pTtcData);

/**
 * @brief 2.2.3.1.3 calculate_decel_for_aeb
 * AEB 모드와 충돌 시간(TTC)을 고려하여 최종 감속도(Decel_AEB_X)를 결정.
 * @param aebMode   (입력) AEB 모드 (Normal, Alert, Brake)
 * @param pTtcData  (입력) TTC 데이터 (TTC, TTC_Brake 등)
 * @return float    (출력) Decel_AEB_X: (-10 ~ 0) [m/s^2]
 */
float calculate_decel_for_aeb(AEB_Mode_e aebMode,
                              const TTC_Data_t *pTtcData);

#ifdef __cplusplus
}
#endif

#endif /* AEB_H */
