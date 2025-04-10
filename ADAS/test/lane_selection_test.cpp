// lane_selection_test.cpp

#include <gtest/gtest.h>
#include <cmath>

extern "C" {
  #include "lane_selection.h"
  #include "adas_shared.h"
}

/*
테스트 항목:
1. Lane Type 및 곡선 여부 판정: 
   - Lane_Curvature가 0보다 크고 LANE_CURVE_THRESHOLD(800m) 미만이면 곡선(true)
   - 그렇지 않으면 false
2. 곡률 전이: 
   - 둘 다 0보다 클 때, |Next_Lane_Curvature - Lane_Curvature| > LANE_CURVE_DIFF_THRESHOLD(400)이면 전이 플래그 true, 아니면 false
3. 진행 방향 오차: 
   - Ego_Heading과 Lane_Heading의 차이를 ±180 범위로 정규화하여 LS_Heading_Error에 저장
4. 차선 중심 오차: 
   - |Lane_Offset| < (Lane_Width/2)면 LS_Is_Within_Lane true, 아니면 false
5. 차로 변경 상태: 
   - Lane_Change_Status가 LANE_CHANGE_KEEP이면 LS_Is_Changing_Lane false, 그 외면 true
6. 인자 NULL 체크: 
   - 입력 포인터 중 하나라도 NULL이면 오류(-1)를 리턴
*/

// Test 1: Lane Type 및 곡선 여부 판정
TEST(LaneSelectionTest, LaneTypeAndCurveTest) {
    LaneData_t laneData;
    laneData.Lane_Type = LANE_TYPE_STRAIGHT; // 입력으로 전달된 차선 유형 그대로 복사됨
    laneData.Lane_Curvature = 500.0f;  // 500 < 800, > 0 → 곡선으로 판단해야 함.
    laneData.Next_Lane_Curvature = 500.0f; // 변화 없음 → 전이 플래그 false.
    laneData.Lane_Offset = 0.0f;
    laneData.Lane_Heading = 0.0f;
    laneData.Lane_Width = 3.5f;
    laneData.Lane_Change_Status = LANE_CHANGE_KEEP;

    // Ego 데이터는 본 테스트에서는 주로 heading만 사용.
    EgoData_t egoData;
    egoData.Ego_Heading = 0.0f;
    egoData.Ego_Position_X = 0.0f;
    egoData.Ego_Position_Y = 0.0f;
    egoData.Ego_Position_Z = 0.0f;

    LaneSelectOutput_t laneOut;
    int ret = LaneSelection_Update(&laneData, &egoData, &laneOut);
    EXPECT_EQ(ret, 0);
    EXPECT_EQ(laneOut.LS_Lane_Type, laneData.Lane_Type);
    EXPECT_TRUE(laneOut.LS_Is_Curved_Lane);
    EXPECT_FALSE(laneOut.LS_Curve_Transition_Flag);
}

// Test 2: 곡률 전이 판단
TEST(LaneSelectionTest, CurvatureTransitionTest) {
    LaneData_t laneData;
    laneData.Lane_Type = LANE_TYPE_CURVE;
    laneData.Lane_Curvature = 600.0f;
    laneData.Next_Lane_Curvature = 1200.0f; // |1200 - 600| = 600 > 400 → 전이 플래그 true.
    laneData.Lane_Offset = 0.0f;
    laneData.Lane_Heading = 0.0f;
    laneData.Lane_Width = 3.5f;
    laneData.Lane_Change_Status = LANE_CHANGE_KEEP;

    EgoData_t egoData;
    egoData.Ego_Heading = 0.0f;
    egoData.Ego_Position_X = 0.0f;
    egoData.Ego_Position_Y = 0.0f;
    egoData.Ego_Position_Z = 0.0f;

    LaneSelectOutput_t laneOut;
    int ret = LaneSelection_Update(&laneData, &egoData, &laneOut);
    EXPECT_EQ(ret, 0);
    EXPECT_TRUE(laneOut.LS_Curve_Transition_Flag);
}

// Test 3: 진행 방향 오차 및 차선 중심 오차 계산
TEST(LaneSelectionTest, HeadingAndOffsetTest) {
    LaneData_t laneData;
    laneData.Lane_Type = LANE_TYPE_STRAIGHT;
    // Lane_Curvature 값이 900이면 (>= 800) → 직선으로 판단
    laneData.Lane_Curvature = 900.0f;
    laneData.Next_Lane_Curvature = 900.0f;
    laneData.Lane_Offset = 1.0f;         // 1.0 < 3.5/2 = 1.75 → 차량이 차선 내에 있음
    laneData.Lane_Heading = 10.0f;
    laneData.Lane_Width = 3.5f;
    laneData.Lane_Change_Status = LANE_CHANGE_KEEP;

    // Ego 차량의 heading이 200이면, heading 차 = 200 - 10 = 190 → 정규화하면 190-360 = -170 (190 > 180)
    EgoData_t egoData;
    egoData.Ego_Heading = 200.0f;
    egoData.Ego_Position_X = 0.0f;
    egoData.Ego_Position_Y = 0.0f;
    egoData.Ego_Position_Z = 0.0f;

    LaneSelectOutput_t laneOut;
    int ret = LaneSelection_Update(&laneData, &egoData, &laneOut);
    EXPECT_EQ(ret, 0);
    EXPECT_NEAR(laneOut.LS_Heading_Error, -170.0f, 0.01f);
    EXPECT_TRUE(laneOut.LS_Is_Within_Lane);
}

// Test 4: 차로 변경 상태 판단
TEST(LaneSelectionTest, LaneChangeStatusTest) {
    LaneData_t laneData;
    laneData.Lane_Type = LANE_TYPE_STRAIGHT;
    laneData.Lane_Curvature = 1000.0f;
    laneData.Next_Lane_Curvature = 1000.0f;
    laneData.Lane_Offset = 0.0f;
    laneData.Lane_Heading = 0.0f;
    laneData.Lane_Width = 3.5f;
    // 차로 변경 중일 때 (LANE_CHANGE_KEEP가 아니면 변경 상태 true)
    laneData.Lane_Change_Status = LANE_CHANGE_CHANGING;

    EgoData_t egoData;
    egoData.Ego_Heading = 0.0f;
    egoData.Ego_Position_X = 0.0f;
    egoData.Ego_Position_Y = 0.0f;
    egoData.Ego_Position_Z = 0.0f;

    LaneSelectOutput_t laneOut;
    int ret = LaneSelection_Update(&laneData, &egoData, &laneOut);
    EXPECT_EQ(ret, 0);
    EXPECT_TRUE(laneOut.LS_Is_Changing_Lane);
}

// Test 5: 유효하지 않은 입력 인자 처리
TEST(LaneSelectionTest, InvalidInputTest) {
    LaneData_t laneData;
    EgoData_t egoData;
    LaneSelectOutput_t laneOut;

    // 세 포인터 중 하나라도 NULL이면 -1을 리턴해야 함.
    EXPECT_LT(LaneSelection_Update(nullptr, &egoData, &laneOut), 0);
    EXPECT_LT(LaneSelection_Update(&laneData, nullptr, &laneOut), 0);
    EXPECT_LT(LaneSelection_Update(&laneData, &egoData, nullptr), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
