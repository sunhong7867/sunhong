// target_selection_test.cpp

#include <gtest/gtest.h>
#include <cmath>
#include "adas_shared.h"

extern "C" {
  #include "target_selection.h"
}

TEST(TargetSelectionTest, SelectTargetFromObjectList_NormalFiltering) {
    // 여러 객체 데이터를 준비
    ObjectData_t objList[3] = {
      {
        .Object_ID = 1,
        .Object_Type = OBJTYPE_CAR,           // 필요한 경우 적절한 enum 값 할당
        .Position_X = 30.0f,
        .Position_Y = 0.0f,
        .Position_Z = 0.0f,
        .Velocity_X = 8.0f,
        .Velocity_Y = 0.0f,
        .Accel_X = 0.0f,
        .Accel_Y = 0.0f,
        .Heading = 0.0f,
        .Distance = 30.0f,
        .Object_Status = OBJSTAT_MOVING,
        .Object_Cell_ID = 0
      },
      {
        .Object_ID = 2,
        .Object_Type = OBJTYPE_PEDESTRIAN,    // 예시
        .Position_X = 250.0f,
        .Position_Y = 0.0f,
        .Position_Z = 0.0f,
        .Velocity_X = 5.0f,
        .Velocity_Y = 0.0f,
        .Accel_X = 0.0f,
        .Accel_Y = 0.0f,
        .Heading = 0.0f,
        .Distance = 250.0f,
        .Object_Status = OBJSTAT_MOVING,
        .Object_Cell_ID = 0
      },
      {
        .Object_ID = 3,
        .Object_Type = OBJTYPE_CAR,           // 예시
        .Position_X = 50.0f,
        .Position_Y = 2.0f,
        .Position_Z = 0.0f,
        .Velocity_X = 3.0f,
        .Velocity_Y = 0.0f,
        .Accel_X = 0.0f,
        .Accel_Y = 0.0f,
        .Heading = 10.0f,
        .Distance = 50.0f,
        .Object_Status = OBJSTAT_MOVING,
        .Object_Cell_ID = 0
      }
  };
    
    EgoData_t egoData = { .Ego_Velocity_X = 10.0f };
    LaneSelectOutput_t lsData;
    lsData.LS_Lane_Width = 3.5f;
    lsData.LS_Heading_Error = 0.0f;
    lsData.LS_Lane_Offset = 0.0f;
    lsData.LS_Is_Curved_Lane = false; // 직선 차선
    // 나머지 값들은 기본값 0으로 설정됨

    FilteredObject_t filteredList[5] = {0};
    int filteredCount = select_target_from_object_list(objList, 3, &egoData, &lsData, filteredList, 5);

    // 예를 들어, 250m인 객체는 필터링 되어야 하므로, filteredCount는 2여야 함.
    EXPECT_EQ(filteredCount, 2);
    
    // 특정 객체의 셀 번호나 상태 검증 등 추가 검증 가능
}

TEST(TargetSelectionTest, PredictObjectFuturePath_PredictionTest) {
    // 필터링된 객체 데이터를 준비
    FilteredObject_t fObj;
    fObj.Filtered_Object_ID = 1;
    fObj.Filtered_Position_X = 30.0f;
    fObj.Filtered_Position_Y = 0.0f;
    fObj.Filtered_Position_Z = 0.0f;
    fObj.Filtered_Velocity_X = 8.0f;
    fObj.Filtered_Velocity_Y = 0.0f;
    fObj.Filtered_Accel_X = 0.0f;
    fObj.Filtered_Accel_Y = 0.0f;
    fObj.Filtered_Heading = 0.0f;
    fObj.Filtered_Distance = 30.0f;
    fObj.Filtered_Object_Status = OBJSTAT_MOVING;
    fObj.Filtered_Object_Cell_ID = 1;
    
    FilteredObject_t filteredList[1] = { fObj };
    PredictedObject_t predList[1] = {0};
    
    LaneData_t laneWp;
    laneWp.Lane_Curvature = 1000.0f; // 직선 차선
    LaneSelectOutput_t lsData;
    lsData.LS_Lane_Offset = 0.0f;
    lsData.LS_Lane_Width = 3.5f;
    
    int predCount = predict_object_future_path(filteredList, 1, &laneWp, &lsData, predList, 1);
    EXPECT_EQ(predCount, 1);
    
    // 3초 후의 등속 예측: 30 + 8*3 = 54
    EXPECT_NEAR(predList[0].Predicted_Position_X, 54.0f, 0.01f);
}

TEST(TargetSelectionTest, SelectTargetsForAccAeb_NormalCase) {
  // 예측 객체 하나를 생성합니다.
  PredictedObject_t predObj = {0}; // 구조체 전체 0으로 초기화 (초기화 순서에 주의)
  predObj.Predicted_Object_ID = 1;
  predObj.Predicted_Position_X = 40.0f;
  predObj.Predicted_Position_Y = 0.0f;
  predObj.Predicted_Distance = 40.0f;
  // 여기서 객체 타입를 명시적으로 설정해야 조건에 부합함.
  predObj.Predicted_Object_Type = OBJTYPE_CAR; // OBJTYPE_CAR로 설정
  predObj.Predicted_Object_Status = OBJSTAT_MOVING;
  predObj.Predicted_Object_Cell_ID = 2;
  predObj.Predicted_Heading = 0.0f;
  predObj.Predicted_Velocity_X = 8.0f;
  predObj.Predicted_Velocity_Y = 0.0f;
  predObj.Predicted_Accel_X = 0.0f;
  predObj.Predicted_Accel_Y = 0.0f;
  predObj.CutIn_Flag = false;
  predObj.CutOut_Flag = false;
  
  PredictedObject_t predList[1] = { predObj };
  EgoData_t egoData = {0};
  egoData.Ego_Velocity_X = 10.0f;
  
  LaneSelectOutput_t lsData = {};
  lsData.LS_Is_Curved_Lane = false;
  lsData.LS_Lane_Offset = 0.0f;
  lsData.LS_Lane_Width = 3.5f;
  
  ACC_Target_t accTarget = {0};
  AEB_Target_t aebTarget = {0};
  
  select_targets_for_acc_aeb(&egoData, predList, 1, &lsData, &accTarget, &aebTarget);
  
  // 조건이 충족되면 ACC 타겟이 선택되어야 하므로, ACC_Target_ID가 1이어야 합니다.
  EXPECT_EQ(accTarget.ACC_Target_ID, 1);
  // AEB도 동일하게 선정되어야 할 경우 검증
  EXPECT_EQ(aebTarget.AEB_Target_ID, 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
