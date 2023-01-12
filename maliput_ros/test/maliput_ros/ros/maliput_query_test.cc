// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput_ros/ros/maliput_query.h"

#include <optional>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <maliput/api/road_network.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_ros/ros/maliput_mock.h"

namespace maliput_ros {
namespace ros {
namespace test {
namespace {

using ::testing::Return;
using ::testing::ReturnRef;

// Test class for MaliputQuery.
class MaliputQueryTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto road_geometry = std::make_unique<RoadGeometryMock>();
    road_geometry_ptr_ = road_geometry.get();
    auto road_rulebook = std::make_unique<RoadRulebookMock>();
    auto traffic_light_book = std::make_unique<TrafficLightBookMock>();
    auto intersection_book = std::make_unique<IntersectionBookMock>();
    auto phase_ring_book = std::make_unique<PhaseRingBookMock>();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    auto right_of_way_rule_state_provider = std::make_unique<RightOfWayRuleStateProviderMock>();
#pragma GCC diagnostic pop
    auto phase_provider = std::make_unique<PhaseProviderMock>();
    auto discrete_value_rule_state_provider = std::make_unique<DiscreteValueRuleStateProviderMock>();
    auto range_value_rule_state_provider = std::make_unique<RangeValueRuleStateProviderMock>();

    auto road_network = std::make_unique<maliput::api::RoadNetwork>(
        std::move(road_geometry), std::move(road_rulebook), std::move(traffic_light_book), std::move(intersection_book),
        std::move(phase_ring_book), std::move(right_of_way_rule_state_provider), std::move(phase_provider),
        std::make_unique<maliput::api::rules::RuleRegistry>(), std::move(discrete_value_rule_state_provider),
        std::move(range_value_rule_state_provider));

    dut_ = std::make_unique<MaliputQuery>(std::move(road_network));
  }

  RoadGeometryMock* road_geometry_ptr_{};
  std::unique_ptr<MaliputQuery> dut_;
};

// Makes sure the maliput::api::RoadNetwork is not nullptr when constructing MaliputQuery.
TEST_F(MaliputQueryTest, ConstructorValidation) {
  std::unique_ptr<maliput::api::RoadNetwork> road_network{};
  ASSERT_THROW({ MaliputQuery(std::move(road_network)); }, maliput::common::assertion_error);
}

// Validates the maliput::api::RoadGeometry pointer.
TEST_F(MaliputQueryTest, RoadGeometry) {
  ASSERT_EQ(dut_->road_geometry(), static_cast<const maliput::api::RoadGeometry*>(road_geometry_ptr_));
}

// Validates MaliputQuery redirects the query via the IdIndex.
TEST_F(MaliputQueryTest, GetJunctionById) {
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  const JunctionMock junction;
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetJunction(kJunctionId)).WillRepeatedly(Return(&junction));
  EXPECT_CALL(*road_geometry_ptr_, DoById()).WillRepeatedly(ReturnRef(id_index));

  ASSERT_EQ(dut_->GetJunctionBy(kJunctionId), &junction);
}

// Validates MaliputQuery redirects the query via the IdIndex.
TEST_F(MaliputQueryTest, GetSegmentById) {
  const maliput::api::SegmentId kSegmentId{"segment_id"};
  const SegmentMock segment;
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetSegment(kSegmentId)).WillRepeatedly(Return(&segment));
  EXPECT_CALL(*road_geometry_ptr_, DoById()).WillRepeatedly(ReturnRef(id_index));

  ASSERT_EQ(dut_->GetSegmentBy(kSegmentId), &segment);
}

// Validates MaliputQuery redirects the query via the IdIndex.
TEST_F(MaliputQueryTest, GetLaneById) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  const LaneMock lane;
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(&lane));
  EXPECT_CALL(*road_geometry_ptr_, DoById()).WillRepeatedly(ReturnRef(id_index));

  ASSERT_EQ(dut_->GetLaneBy(kLaneId), &lane);
}

// Validates MaliputQuery redirects the query via the IdIndex.
TEST_F(MaliputQueryTest, GetBranchPointById) {
  const maliput::api::BranchPointId kBranchPointId{"branch_point_id"};
  const BranchPointMock branch_point;
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetBranchPoint(kBranchPointId)).WillRepeatedly(Return(&branch_point));
  EXPECT_CALL(*road_geometry_ptr_, DoById()).WillRepeatedly(ReturnRef(id_index));

  ASSERT_EQ(dut_->GetBranchPointBy(kBranchPointId), &branch_point);
}

// Validates MaliputQuery redirects ther query through the RoadGeometry.
TEST_F(MaliputQueryTest, ToRoadPosition) {
  const LaneMock lane;
  const maliput::api::LanePosition kLanePosition{1., 2., 3.};
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  const maliput::api::InertialPosition kInertialPosition{4., 5., 6.};
  constexpr const double kDistance{7.};
  const maliput::api::RoadPositionResult kRoadPositionResult{kRoadPosition, kInertialPosition, kDistance};
  EXPECT_CALL(*road_geometry_ptr_, DoToRoadPosition(kInertialPosition, ::testing::_))
      .WillRepeatedly(Return(kRoadPositionResult));

  const maliput::api::RoadPositionResult result = dut_->ToRoadPosition(kInertialPosition);

  ASSERT_TRUE(maliput::api::test::IsRoadPositionResultClose(result, kRoadPositionResult, 0. /* tolerance */));
}

// Validates MaliputQuery redirects ther query through the RoadGeometry.
TEST_F(MaliputQueryTest, FindRoadPositions) {
  const LaneMock lane;
  const maliput::api::InertialPosition kInertialPosition{4., 5., 6.};
  constexpr const double kRadius{8.};
  const maliput::api::LanePosition kLanePosition{1., 2., 3.};
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  constexpr const double kDistance{7.};
  const maliput::api::RoadPositionResult kRoadPositionResult{kRoadPosition, kInertialPosition, kDistance};
  const std::vector<maliput::api::RoadPositionResult> kRoadPositionResults{kRoadPositionResult};
  EXPECT_CALL(*road_geometry_ptr_, DoFindRoadPositions(kInertialPosition, kRadius))
      .WillRepeatedly(Return(kRoadPositionResults));

  const std::vector<maliput::api::RoadPositionResult> result = dut_->FindRoadPositions(kInertialPosition, kRadius);

  ASSERT_EQ(result.size(), 1u);
  ASSERT_TRUE(maliput::api::test::IsRoadPositionResultClose(result[0], kRoadPositionResult, 0. /* tolerance */));
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
