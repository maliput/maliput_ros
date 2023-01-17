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
  static constexpr double kZeroTolerance{0.};

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

  ASSERT_TRUE(maliput::api::test::IsRoadPositionResultClose(result, kRoadPositionResult, kZeroTolerance));
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
  ASSERT_TRUE(maliput::api::test::IsRoadPositionResultClose(result[0], kRoadPositionResult, kZeroTolerance));
}

// Validates MaliputQuery redirects ther query through the Lane.
TEST_F(MaliputQueryTest, ToInertialPoseWithValidValues) {
  const LaneMock lane;
  const maliput::api::LanePosition kLanePosition{1., 2., 3.};
  const maliput::api::InertialPosition kInertialPosition{4., 5., 6.};
  EXPECT_CALL(lane, DoToInertialPosition(::testing::_)).WillRepeatedly(Return(kInertialPosition));
  // A quaternion that rotates x->y, y->z, z->x...
  const maliput::math::Quaternion kQuaternion =
      maliput::math::Quaternion(M_PI * 2. / 3., maliput::math::Vector3(1.0, 1.0, 1.0).normalized());
  const maliput::api::Rotation kRotation = maliput::api::Rotation::FromQuat(kQuaternion);
  EXPECT_CALL(lane, DoGetOrientation(::testing::_)).WillRepeatedly(Return(kRotation));
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};

  const std::optional<std::pair<maliput::api::InertialPosition, maliput::api::Rotation>> result =
      dut_->ToInertialPose(kRoadPosition);

  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(maliput::api::test::IsInertialPositionClose(result->first, kInertialPosition, kZeroTolerance));
  ASSERT_TRUE(maliput::api::test::IsRotationClose(result->second, kRotation, kZeroTolerance));
}

// Validates MaliputQuery returns std::nullopt when lane is nullptr.
TEST_F(MaliputQueryTest, ToInertialPoseWithInvalidRoadPosition) {
  const maliput::api::RoadPosition kRoadPosition;

  const std::optional<std::pair<maliput::api::InertialPosition, maliput::api::Rotation>> result =
      dut_->ToInertialPose(kRoadPosition);

  ASSERT_FALSE(result.has_value());
}

// Validates MaliputQuery redirects ther query through the Lane and is valid.
TEST_F(MaliputQueryTest, EvalMotionDerivativesWithValidRoadPosition) {
  const LaneMock lane;
  const maliput::api::LanePosition kMotionDerivatives{1., 2., 3.};
  EXPECT_CALL(lane, DoEvalMotionDerivatives(::testing::_, ::testing::_)).WillRepeatedly(Return(kMotionDerivatives));
  const maliput::api::LanePosition kLanePosition{4., 5., 6.};
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  const maliput::api::IsoLaneVelocity kVelocity{7., 8., 9.};

  const maliput::api::LanePosition result = dut_->EvalMotionDerivatives(kRoadPosition, kVelocity);

  ASSERT_TRUE(maliput::api::test::IsLanePositionClose(result, kMotionDerivatives, kZeroTolerance));
}

// Validates MaliputQuery returns a zero maliput::api::LanePosition when lane is nullptr.
TEST_F(MaliputQueryTest, EvalMotionDerivativesWithInvalidRoadPosition) {
  const maliput::api::RoadPosition kRoadPosition;
  const maliput::api::IsoLaneVelocity kVelocity{7., 8., 9.};

  const maliput::api::LanePosition result = dut_->EvalMotionDerivatives(kRoadPosition, kVelocity);

  ASSERT_TRUE(maliput::api::test::IsLanePositionClose(result, maliput::api::LanePosition{}, kZeroTolerance));
}

// Validates MaliputQuery redirects boundaries queries through to the Lane.
TEST_F(MaliputQueryTest, EvalLaneBoundariesWithValidRoadPosition) {
  const LaneMock lane;
  static constexpr double kS = 1.;
  static constexpr double kR = 2.;
  const maliput::api::RBounds kLaneBounds{-4., 5.};
  EXPECT_CALL(lane, do_lane_bounds(kS)).WillRepeatedly(Return(kLaneBounds));
  const maliput::api::RBounds kSegmentBounds{-6., 7.};
  EXPECT_CALL(lane, do_segment_bounds(kS)).WillRepeatedly(Return(kSegmentBounds));
  const maliput::api::HBounds kElevationBounds{-8., 9.};
  EXPECT_CALL(lane, do_elevation_bounds(kS, kR)).WillRepeatedly(Return(kElevationBounds));
  const maliput::api::LanePosition kLanePosition{kS, kR, 3.};
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};

  const std::optional<MaliputQuery::LaneBoundaries> result = dut_->EvalLaneBoundaries(kRoadPosition);

  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(maliput::api::test::IsRBoundsClose(result->lane_boundaries, kLaneBounds, kZeroTolerance));
  ASSERT_TRUE(maliput::api::test::IsRBoundsClose(result->segment_boundaries, kSegmentBounds, kZeroTolerance));
  ASSERT_TRUE(maliput::api::test::IsHBoundsClose(result->elevation_boundaries, kElevationBounds, kZeroTolerance));
}

// Validates MaliputQuery redirects boundaries queries through to the Lane.
TEST_F(MaliputQueryTest, EvalLaneBoundariesWithInvalidRoadPosition) {
  const maliput::api::RoadPosition kRoadPosition;

  const std::optional<MaliputQuery::LaneBoundaries> result = dut_->EvalLaneBoundaries(kRoadPosition);

  ASSERT_FALSE(result.has_value());
}

TEST_F(MaliputQueryTest, DeriveLaneSRoutesValidInput) {
  // RoadPositions are in the same Lane to shortcircuit the search and avoid mocking an entire RoadGeometry.
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LanePosition kStartLanePosition{1., 0., 0.};
  const maliput::api::LanePosition kEndLanePosition{2., 0., 0.};
  const maliput::api::RoadPosition kStartRoadPosition{&lane, kStartLanePosition};
  const maliput::api::RoadPosition kEndRoadPosition{&lane, kEndLanePosition};
  static constexpr double kMaxLengthM{1e6};

  const std::vector<maliput::api::LaneSRoute> result =
      dut_->DeriveLaneSRoutes(kStartRoadPosition, kEndRoadPosition, kMaxLengthM);

  ASSERT_EQ(result.size(), 1u);
  ASSERT_EQ(result[0].ranges().size(), 1u);
  ASSERT_EQ(result[0].ranges()[0].lane_id(), kLaneId);
  ASSERT_EQ(result[0].ranges()[0].s_range().s0(), kStartLanePosition.s());
  ASSERT_EQ(result[0].ranges()[0].s_range().s1(), kEndLanePosition.s());
}

TEST_F(MaliputQueryTest, DeriveLaneSRoutesInvalidInput) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  const maliput::api::LanePosition kStartLanePosition{1., 0., 0.};
  const maliput::api::LanePosition kEndLanePosition{2., 0., 0.};
  const maliput::api::RoadPosition kInvalidRoadPosition;
  const maliput::api::RoadPosition kStartRoadPosition{&lane, kEndLanePosition};
  const maliput::api::RoadPosition kEndRoadPosition{&lane, kEndLanePosition};
  static constexpr double kMaxLengthM{1e6};
  static constexpr double kInvalidMaxLengthM{-1.};

  ASSERT_TRUE(dut_->DeriveLaneSRoutes(kInvalidRoadPosition, kEndRoadPosition, kMaxLengthM).empty());
  ASSERT_TRUE(dut_->DeriveLaneSRoutes(kStartRoadPosition, kInvalidRoadPosition, kMaxLengthM).empty());
  ASSERT_TRUE(dut_->DeriveLaneSRoutes(kStartRoadPosition, kEndRoadPosition, kInvalidMaxLengthM).empty());
}

TEST_F(MaliputQueryTest, SampleAheadWaypointsValidInput) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::LaneSRange kLaneSRange{kLaneId, maliput::api::SRange{10., 20.}};
  const maliput::api::LaneSRoute kLaneSRoute{{kLaneSRange}};
  static constexpr double kPathLengthSamplingRate{5.};
  const maliput::api::InertialPosition kInertialPosition1{10., 5., 1.};
  const maliput::api::InertialPosition kInertialPosition2{15., 5., 1.};
  const maliput::api::InertialPosition kInertialPosition3{20., 5., 1.};
  const std::vector<maliput::api::InertialPosition> kExpectedResult{kInertialPosition1, kInertialPosition2,
                                                                    kInertialPosition3};
  EXPECT_CALL(*road_geometry_ptr_, DoSampleAheadWaypoints(::testing::_, kPathLengthSamplingRate))
      .WillRepeatedly(Return(kExpectedResult));

  const std::vector<maliput::api::InertialPosition> result =
      dut_->SampleAheadWaypoints(kLaneSRoute, kPathLengthSamplingRate);

  ASSERT_EQ(result.size(), 3u);
  ASSERT_TRUE(maliput::api::test::IsInertialPositionClose(result[0], kInertialPosition1, kZeroTolerance));
  ASSERT_TRUE(maliput::api::test::IsInertialPositionClose(result[1], kInertialPosition2, kZeroTolerance));
  ASSERT_TRUE(maliput::api::test::IsInertialPositionClose(result[2], kInertialPosition3, kZeroTolerance));
}

TEST_F(MaliputQueryTest, SampleAheadWaypointsInValidInput) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::LaneSRange kLaneSRange{kLaneId, maliput::api::SRange{10., 20.}};
  const maliput::api::LaneSRoute kLaneSRoute{{kLaneSRange}};
  static constexpr double kPathLengthSamplingRate{-1.};

  const std::vector<maliput::api::InertialPosition> result =
      dut_->SampleAheadWaypoints(kLaneSRoute, kPathLengthSamplingRate);

  ASSERT_TRUE(result.empty());
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
