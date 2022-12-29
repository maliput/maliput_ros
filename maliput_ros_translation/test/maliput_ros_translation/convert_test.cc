// BSD 3-Clause License
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
#include "maliput_ros_translation/convert.h"

#include <optional>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <maliput/test_utilities/mock.h>

#include "maliput/test_utilities/mock_geometry.h"

namespace maliput_ros_translation {
namespace test {
namespace {

using ::testing::Return;
using ::testing::ReturnRef;

class RoadGeometryMock final : public maliput::api::test::MockRoadGeometry {
 public:
  explicit RoadGeometryMock(const maliput::api::RoadGeometryId& id) : maliput::api::test::MockRoadGeometry(id) {}
  MOCK_CONST_METHOD0(do_id, maliput::api::RoadGeometryId());
  MOCK_CONST_METHOD0(do_num_junctions, int());
  MOCK_CONST_METHOD1(do_junction, const maliput::api::Junction*(int));
  MOCK_CONST_METHOD0(do_num_branch_points, int());
  MOCK_CONST_METHOD1(do_branch_point, const maliput::api::BranchPoint*(int));
  MOCK_CONST_METHOD0(do_linear_tolerance, double());
  MOCK_CONST_METHOD0(do_angular_tolerance, double());
  MOCK_CONST_METHOD0(do_scale_length, double());
  MOCK_CONST_METHOD0(do_inertial_to_backend_frame_translation, maliput::math::Vector3());
};

class BranchPointMock final : public maliput::api::BranchPoint {
 public:
  MOCK_CONST_METHOD0(do_id, maliput::api::BranchPointId());
  MOCK_CONST_METHOD0(do_road_geometry, const maliput::api::RoadGeometry*());
  MOCK_CONST_METHOD1(DoGetConfluentBranches, const maliput::api::LaneEndSet*(const maliput::api::LaneEnd& end));
  MOCK_CONST_METHOD1(DoGetOngoingBranches, const maliput::api::LaneEndSet*(const maliput::api::LaneEnd& end));
  MOCK_CONST_METHOD1(DoGetDefaultBranch, std::optional<maliput::api::LaneEnd>(const maliput::api::LaneEnd& end));
  MOCK_CONST_METHOD0(DoGetASide, const maliput::api::LaneEndSet*());
  MOCK_CONST_METHOD0(DoGetBSide, const maliput::api::LaneEndSet*());
};

class LaneMock final : public maliput::geometry_base::test::MockLane {
 public:
  explicit LaneMock(const maliput::api::LaneId& id) : MockLane(id) {}
  MOCK_CONST_METHOD0(do_id, maliput::api::LaneId());
  MOCK_CONST_METHOD0(do_index, int());
  MOCK_CONST_METHOD0(do_segment, const maliput::api::Segment*());
  MOCK_CONST_METHOD0(do_to_left, const maliput::api::Lane*());
  MOCK_CONST_METHOD0(do_to_right, const maliput::api::Lane*());
  MOCK_CONST_METHOD0(do_length, double());
  MOCK_CONST_METHOD1(DoGetBranchPoint, const maliput::api::BranchPoint*(const maliput::api::LaneEnd::Which));
  MOCK_CONST_METHOD1(DoGetDefaultBranch, std::optional<maliput::api::LaneEnd>(const maliput::api::LaneEnd::Which));
};

class LaneEndSetMock final : public maliput::api::LaneEndSet {
 public:
  MOCK_CONST_METHOD0(do_size, int());
  MOCK_CONST_METHOD1(do_get, const maliput::api::LaneEnd&(int));
};

class SegmentMock final : public maliput::api::Segment {
 public:
  MOCK_CONST_METHOD0(do_id, maliput::api::SegmentId());
  MOCK_CONST_METHOD0(do_junction, const maliput::api::Junction*());
  MOCK_CONST_METHOD0(do_num_lanes, int());
  MOCK_CONST_METHOD1(do_lane, const maliput::api::Lane*(int));
};

class JunctionMock final : public maliput::api::Junction {
 public:
  MOCK_CONST_METHOD0(do_id, maliput::api::JunctionId());
  MOCK_CONST_METHOD0(do_road_geometry, const maliput::api::RoadGeometry*());
  MOCK_CONST_METHOD0(do_num_segments, int());
  MOCK_CONST_METHOD1(do_segment, const maliput::api::Segment*(int));
};

class IdsToRosMessageTest : public ::testing::Test {
 public:
  const maliput::api::BranchPointId kBranchPointId{"branch_point_id"};
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};
  const maliput::api::SegmentId kSegmentId{"segment_id"};
};

TEST_F(IdsToRosMessageTest, BranchPointId) {
  const maliput_ros_interfaces::msg::BranchPointId dut = ToRosMessage(kBranchPointId);
  ASSERT_EQ(dut.id, kBranchPointId.string());
}

TEST_F(IdsToRosMessageTest, LaneId) {
  const maliput_ros_interfaces::msg::LaneId dut = ToRosMessage(kLaneId);
  ASSERT_EQ(dut.id, kLaneId.string());
}

TEST_F(IdsToRosMessageTest, JunctionId) {
  const maliput_ros_interfaces::msg::JunctionId dut = ToRosMessage(kJunctionId);
  ASSERT_EQ(dut.id, kJunctionId.string());
}

TEST_F(IdsToRosMessageTest, RoadGeometryId) {
  const maliput_ros_interfaces::msg::RoadGeometryId dut = ToRosMessage(kRoadGeometryId);
  ASSERT_EQ(dut.id, kRoadGeometryId.string());
}

TEST_F(IdsToRosMessageTest, kSegmentId) {
  const maliput_ros_interfaces::msg::SegmentId dut = ToRosMessage(kSegmentId);
  ASSERT_EQ(dut.id, kSegmentId.string());
}

class LaneEndToRosMessageTest : public ::testing::Test {
 public:
  const maliput::api::LaneId kLaneId{"lane_id"};
  const std::string kNullPtrLaneId{""};
};

TEST_F(LaneEndToRosMessageTest, LaneEndAtStart) {
  LaneMock lane(kLaneId);
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LaneEnd lane_end(&lane, maliput::api::LaneEnd::Which::kStart);

  const maliput_ros_interfaces::msg::LaneEnd dut = ToRosMessage(lane_end);

  ASSERT_EQ(dut.lane_id.id, kLaneId.string());
  ASSERT_EQ(dut.end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
}

TEST_F(LaneEndToRosMessageTest, LaneEndAtFinish) {
  LaneMock lane(kLaneId);
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LaneEnd lane_end(&lane, maliput::api::LaneEnd::Which::kFinish);

  const maliput_ros_interfaces::msg::LaneEnd dut = ToRosMessage(lane_end);

  ASSERT_EQ(dut.lane_id.id, kLaneId.string());
  ASSERT_EQ(dut.end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(LaneEndToRosMessageTest, NullLane) {
  const maliput_ros_interfaces::msg::LaneEnd dut = ToRosMessage(maliput::api::LaneEnd());

  ASSERT_EQ(dut.lane_id.id, kNullPtrLaneId);
}

GTEST_TEST(LaneEndSetToRosMessageTest, ValidLaneEndSet) {
  static constexpr int kSize{2};
  const maliput::api::LaneId kLaneIdA{"lane_id_a"};
  const maliput::api::LaneId kLaneIdB{"lane_id_b"};
  const maliput::api::LaneEnd::Which kWhichA{maliput::api::LaneEnd::Which::kStart};
  const maliput::api::LaneEnd::Which kWhichB{maliput::api::LaneEnd::Which::kFinish};
  LaneMock lane_a(kLaneIdA);
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  LaneMock lane_b(kLaneIdB);
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  const maliput::api::LaneEnd lane_end_a(&lane_a, kWhichA);
  const maliput::api::LaneEnd lane_end_b(&lane_b, kWhichB);
  LaneEndSetMock lane_end_set;
  EXPECT_CALL(lane_end_set, do_size()).WillRepeatedly(Return(kSize));
  EXPECT_CALL(lane_end_set, do_get(0)).WillRepeatedly(ReturnRef(lane_end_a));
  EXPECT_CALL(lane_end_set, do_get(1)).WillRepeatedly(ReturnRef(lane_end_b));

  const maliput_ros_interfaces::msg::LaneEndSet dut = ToRosMessage(&lane_end_set);

  ASSERT_EQ(dut.lane_ends.size(), static_cast<size_t>(kSize));
  ASSERT_EQ(dut.lane_ends[0].lane_id.id, kLaneIdA.string());
  ASSERT_EQ(dut.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.lane_ends[1].lane_id.id, kLaneIdB.string());
  ASSERT_EQ(dut.lane_ends[1].end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

GTEST_TEST(LaneEndSetToRosMessageTest, NullptrLaneEndSet) {
  const maliput_ros_interfaces::msg::LaneEndSet dut =
      ToRosMessage(static_cast<const maliput::api::LaneEndSet*>(nullptr));

  ASSERT_EQ(dut.lane_ends.size(), static_cast<size_t>(0));
}

class BranchPointToRosMessageTest : public ::testing::Test {
 public:
  const maliput::api::LaneId kLaneIdA{"lane_id_a"};
  const maliput::api::LaneId kLaneIdB{"lane_id_b"};
  const maliput::api::LaneEnd::Which kWhichA{maliput::api::LaneEnd::Which::kStart};
  const maliput::api::LaneEnd::Which kWhichB{maliput::api::LaneEnd::Which::kFinish};
  const maliput::api::RoadGeometryId kRoadGeometryId{"kRoadGeometryId"};
  const maliput::api::BranchPointId kBranchPointId{"branch_point_id"};
  const std::string kNullBranchPointId{""};
};

TEST_F(BranchPointToRosMessageTest, ValidBranchPoint) {
  LaneMock lane_a(kLaneIdA);
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  LaneMock lane_b(kLaneIdB);
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  const maliput::api::LaneEnd lane_end_a(&lane_a, kWhichA);
  const maliput::api::LaneEnd lane_end_b(&lane_b, kWhichB);
  LaneEndSetMock lane_end_set_a;
  EXPECT_CALL(lane_end_set_a, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(lane_end_set_a, do_get(0)).WillRepeatedly(ReturnRef(lane_end_a));
  LaneEndSetMock lane_end_set_b;
  EXPECT_CALL(lane_end_set_b, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(lane_end_set_b, do_get(0)).WillRepeatedly(ReturnRef(lane_end_b));
  RoadGeometryMock road_geometry(kRoadGeometryId);
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  BranchPointMock branch_point;
  EXPECT_CALL(branch_point, do_id()).WillRepeatedly(Return(kBranchPointId));
  EXPECT_CALL(branch_point, do_road_geometry()).WillRepeatedly(Return(&road_geometry));
  EXPECT_CALL(branch_point, DoGetASide()).WillRepeatedly(Return(&lane_end_set_a));
  EXPECT_CALL(branch_point, DoGetBSide()).WillRepeatedly(Return(&lane_end_set_b));

  const maliput_ros_interfaces::msg::BranchPoint dut = ToRosMessage(&branch_point);

  ASSERT_EQ(dut.id.id, kBranchPointId.string());
  ASSERT_EQ(dut.road_geometry_id.id, kRoadGeometryId.string());
  ASSERT_EQ(dut.a_side.lane_ends.size(), static_cast<size_t>(1));
  ASSERT_EQ(dut.a_side.lane_ends[0].lane_id.id, kLaneIdA.string());
  ASSERT_EQ(dut.a_side.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.b_side.lane_ends.size(), static_cast<size_t>(1));
  ASSERT_EQ(dut.b_side.lane_ends[0].lane_id.id, kLaneIdB.string());
  ASSERT_EQ(dut.b_side.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(BranchPointToRosMessageTest, BranchPointWithEmptyBSide) {
  LaneMock lane_a(kLaneIdA);
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  const maliput::api::LaneEnd lane_end_a(&lane_a, kWhichA);
  LaneEndSetMock lane_end_set_a;
  EXPECT_CALL(lane_end_set_a, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(lane_end_set_a, do_get(0)).WillRepeatedly(ReturnRef(lane_end_a));
  RoadGeometryMock road_geometry(kRoadGeometryId);
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  BranchPointMock branch_point;
  EXPECT_CALL(branch_point, do_id()).WillRepeatedly(Return(kBranchPointId));
  EXPECT_CALL(branch_point, do_road_geometry()).WillRepeatedly(Return(&road_geometry));
  EXPECT_CALL(branch_point, DoGetASide()).WillRepeatedly(Return(&lane_end_set_a));
  EXPECT_CALL(branch_point, DoGetBSide()).WillRepeatedly(Return(static_cast<const maliput::api::LaneEndSet*>(nullptr)));

  const maliput_ros_interfaces::msg::BranchPoint dut = ToRosMessage(&branch_point);

  ASSERT_EQ(dut.id.id, kBranchPointId.string());
  ASSERT_EQ(dut.road_geometry_id.id, kRoadGeometryId.string());
  ASSERT_EQ(dut.a_side.lane_ends.size(), static_cast<size_t>(1));
  ASSERT_EQ(dut.a_side.lane_ends[0].lane_id.id, kLaneIdA.string());
  ASSERT_EQ(dut.a_side.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.b_side.lane_ends.size(), static_cast<size_t>(0));
}

TEST_F(BranchPointToRosMessageTest, NullBranchPoint) {
  const maliput_ros_interfaces::msg::BranchPoint dut =
      ToRosMessage(static_cast<const maliput::api::BranchPoint*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullBranchPointId);
}

GTEST_TEST(SegmentToRosMessageTest, ValidSegment) {
  static constexpr int kSize{2};
  const maliput::api::LaneId kLaneIdA{"lane_id_a"};
  const maliput::api::LaneId kLaneIdB{"lane_id_b"};
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  const maliput::api::SegmentId kSegmentId{"segment_id"};
  LaneMock lane_a(kLaneIdA);
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  LaneMock lane_b(kLaneIdB);
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  JunctionMock junction;
  EXPECT_CALL(junction, do_id()).WillRepeatedly(Return(kJunctionId));
  SegmentMock segment;
  EXPECT_CALL(segment, do_id()).WillRepeatedly(Return(kSegmentId));
  EXPECT_CALL(segment, do_junction()).WillRepeatedly(Return(&junction));
  EXPECT_CALL(segment, do_num_lanes()).WillRepeatedly(Return(kSize));
  EXPECT_CALL(segment, do_lane(0)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(segment, do_lane(1)).WillRepeatedly(Return(&lane_b));

  const maliput_ros_interfaces::msg::Segment dut = ToRosMessage(&segment);

  ASSERT_EQ(dut.id.id, kSegmentId.string());
  ASSERT_EQ(dut.junction_id.id, kJunctionId.string());
  ASSERT_EQ(dut.lane_ids.size(), static_cast<size_t>(kSize));
  ASSERT_EQ(dut.lane_ids[0].id, kLaneIdA.string());
  ASSERT_EQ(dut.lane_ids[1].id, kLaneIdB.string());
}

GTEST_TEST(SegmentToRosMessageTest, NullSegment) {
  const std::string kNullSegmentId{""};

  const maliput_ros_interfaces::msg::Segment dut = ToRosMessage(static_cast<const maliput::api::Segment*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullSegmentId);
}

GTEST_TEST(JunctionToRosMessageTest, ValidJunction) {
  static constexpr int kSize{2};
  const maliput::api::SegmentId kSgmentIdA{"segment_id_a"};
  const maliput::api::SegmentId kSgmentIdB{"segment_id_b"};
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  SegmentMock segment_a;
  EXPECT_CALL(segment_a, do_id()).WillRepeatedly(Return(kSgmentIdA));
  SegmentMock segment_b;
  EXPECT_CALL(segment_b, do_id()).WillRepeatedly(Return(kSgmentIdB));
  RoadGeometryMock road_geometry(kRoadGeometryId);
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  JunctionMock junction;
  EXPECT_CALL(junction, do_id()).WillRepeatedly(Return(kJunctionId));
  EXPECT_CALL(junction, do_road_geometry()).WillRepeatedly(Return(&road_geometry));
  EXPECT_CALL(junction, do_num_segments()).WillRepeatedly(Return(kSize));
  EXPECT_CALL(junction, do_segment(0)).WillRepeatedly(Return(&segment_a));
  EXPECT_CALL(junction, do_segment(1)).WillRepeatedly(Return(&segment_b));

  const maliput_ros_interfaces::msg::Junction dut = ToRosMessage(&junction);

  ASSERT_EQ(dut.id.id, kJunctionId.string());
  ASSERT_EQ(dut.road_geometry_id.id, kRoadGeometryId.string());
  ASSERT_EQ(dut.segment_ids.size(), static_cast<size_t>(kSize));
  ASSERT_EQ(dut.segment_ids[0].id, kSgmentIdA.string());
  ASSERT_EQ(dut.segment_ids[1].id, kSgmentIdB.string());
}

GTEST_TEST(JunctionToRosMessageTest, NullJunction) {
  const std::string kNullJunctionId{""};

  const maliput_ros_interfaces::msg::Junction dut = ToRosMessage(static_cast<const maliput::api::Junction*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullJunctionId);
}

GTEST_TEST(RoadGeometryToRosMessageTest, ValidRoadGeometry) {
  static constexpr int kSizeJunctions{2};
  static constexpr int kSizeBranchPoints{2};
  static constexpr double kLinearTolerance{1e-12};
  static constexpr double kAngularTolerance{5e-12};
  static constexpr double kScaleLength{1.};
  const maliput::api::JunctionId kJunctionIdA{"junction_id_a"};
  const maliput::api::JunctionId kJunctionIdB{"junction_id_b"};
  const maliput::api::BranchPointId kBranchPointIdA{"branch_point_id_a"};
  const maliput::api::BranchPointId kBranchPointIdB{"branch_point_id_b"};
  const maliput::math::Vector3 kInertialToBackendFrameTranslation{1., 2., 3.};
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};
  JunctionMock junction_a;
  EXPECT_CALL(junction_a, do_id()).WillRepeatedly(Return(kJunctionIdA));
  JunctionMock junction_b;
  EXPECT_CALL(junction_b, do_id()).WillRepeatedly(Return(kJunctionIdB));
  BranchPointMock branch_point_a;
  EXPECT_CALL(branch_point_a, do_id()).WillRepeatedly(Return(kBranchPointIdA));
  BranchPointMock branch_point_b;
  EXPECT_CALL(branch_point_b, do_id()).WillRepeatedly(Return(kBranchPointIdB));
  RoadGeometryMock road_geometry(kRoadGeometryId);
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  EXPECT_CALL(road_geometry, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  EXPECT_CALL(road_geometry, do_angular_tolerance()).WillRepeatedly(Return(kAngularTolerance));
  EXPECT_CALL(road_geometry, do_scale_length()).WillRepeatedly(Return(kScaleLength));
  EXPECT_CALL(road_geometry, do_inertial_to_backend_frame_translation())
      .WillRepeatedly(Return(kInertialToBackendFrameTranslation));
  EXPECT_CALL(road_geometry, do_num_junctions()).WillRepeatedly(Return(kSizeJunctions));
  EXPECT_CALL(road_geometry, do_num_branch_points()).WillRepeatedly(Return(kSizeBranchPoints));
  EXPECT_CALL(road_geometry, do_junction(0)).WillRepeatedly(Return(&junction_a));
  EXPECT_CALL(road_geometry, do_junction(1)).WillRepeatedly(Return(&junction_b));
  EXPECT_CALL(road_geometry, do_branch_point(0)).WillRepeatedly(Return(&branch_point_a));
  EXPECT_CALL(road_geometry, do_branch_point(1)).WillRepeatedly(Return(&branch_point_b));

  const maliput_ros_interfaces::msg::RoadGeometry dut = ToRosMessage(&road_geometry);

  ASSERT_EQ(dut.id.id, kRoadGeometryId.string());
  ASSERT_EQ(dut.linear_tolerance, kLinearTolerance);
  ASSERT_EQ(dut.angular_tolerance, kAngularTolerance);
  ASSERT_EQ(dut.scale_length, kScaleLength);
  ASSERT_EQ(dut.inertial_to_backend_frame_translation.x, kInertialToBackendFrameTranslation.x());
  ASSERT_EQ(dut.inertial_to_backend_frame_translation.y, kInertialToBackendFrameTranslation.y());
  ASSERT_EQ(dut.inertial_to_backend_frame_translation.z, kInertialToBackendFrameTranslation.z());
  ASSERT_EQ(dut.junction_ids.size(), static_cast<size_t>(kSizeJunctions));
  ASSERT_EQ(dut.junction_ids[0].id, kJunctionIdA.string());
  ASSERT_EQ(dut.junction_ids[1].id, kJunctionIdB.string());
  ASSERT_EQ(dut.branch_point_ids.size(), static_cast<size_t>(kSizeJunctions));
  ASSERT_EQ(dut.branch_point_ids[0].id, kBranchPointIdA.string());
  ASSERT_EQ(dut.branch_point_ids[1].id, kBranchPointIdB.string());
}

GTEST_TEST(RoadGeometryToRosMessageTest, NullRoadGeometry) {
  const std::string kNullRoadGeometryId{""};

  const maliput_ros_interfaces::msg::RoadGeometry dut =
      ToRosMessage(static_cast<const maliput::api::RoadGeometry*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullRoadGeometryId);
}

class LaneToRosMessageTest : public ::testing::Test {
 public:
  static constexpr int kIndex{3};
  static constexpr double kLaneLength{123.456};
  const maliput::api::BranchPointId kStartBranchPointId{"start_branch_point_id"};
  const maliput::api::BranchPointId kFinishBranchPointId{"finish_branch_point_id"};
  const maliput::api::SegmentId kSegmentId{"segment_id"};
  const maliput::api::LaneId kLeftLaneId{"left_lane_id"};
  const maliput::api::LaneId kRightLaneId{"right_lane_id"};
  const maliput::api::LaneId kDefaultStartLaneId{"default_start_lane_id"};
  const maliput::api::LaneEnd::Which kDefaultStartWhich{maliput::api::LaneEnd::Which::kStart};
  const maliput::api::LaneId kDefaultFinishLaneId{"default_finish_lane_id"};
  const maliput::api::LaneEnd::Which kDefaultFinishWhich{maliput::api::LaneEnd::Which::kFinish};
  const maliput::api::LaneId kLaneId{"lane_id"};
  const std::string kNullLaneId{""};

  void SetUp() override {
    EXPECT_CALL(segment, do_id()).WillRepeatedly(Return(kSegmentId));
    EXPECT_CALL(left_lane, do_id()).WillRepeatedly(Return(kLeftLaneId));
    EXPECT_CALL(right_lane, do_id()).WillRepeatedly(Return(kRightLaneId));
    EXPECT_CALL(default_start_lane, do_id()).WillRepeatedly(Return(kDefaultStartLaneId));
    EXPECT_CALL(default_finish_lane, do_id()).WillRepeatedly(Return(kDefaultFinishLaneId));
    EXPECT_CALL(start_branch_point, do_id()).WillRepeatedly(Return(kStartBranchPointId));
    EXPECT_CALL(finish_branch_point, do_id()).WillRepeatedly(Return(kFinishBranchPointId));
    EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
    EXPECT_CALL(lane, do_segment()).WillRepeatedly(Return(&segment));
    EXPECT_CALL(lane, do_index()).WillRepeatedly(Return(kIndex));
    EXPECT_CALL(lane, do_to_left()).WillRepeatedly(Return(&left_lane));
    EXPECT_CALL(lane, do_to_right()).WillRepeatedly(Return(&right_lane));
    EXPECT_CALL(lane, do_length()).WillRepeatedly(Return(kLaneLength));
    EXPECT_CALL(lane, DoGetBranchPoint(maliput::api::LaneEnd::Which::kStart))
        .WillRepeatedly(Return(&start_branch_point));
    EXPECT_CALL(lane, DoGetBranchPoint(maliput::api::LaneEnd::Which::kFinish))
        .WillRepeatedly(Return(&finish_branch_point));
  }

  SegmentMock segment;
  LaneMock left_lane{kLeftLaneId};
  LaneMock right_lane{kRightLaneId};
  LaneMock default_start_lane{kDefaultStartLaneId};
  LaneMock default_finish_lane{kDefaultFinishLaneId};
  BranchPointMock start_branch_point;
  BranchPointMock finish_branch_point;
  LaneMock lane{kLaneId};
};

TEST_F(LaneToRosMessageTest, FullLane) {
  const std::optional<maliput::api::LaneEnd> default_branch_start =
      maliput::api::LaneEnd{&default_start_lane, kDefaultStartWhich};
  const std::optional<maliput::api::LaneEnd> default_branch_finish =
      maliput::api::LaneEnd{&default_finish_lane, kDefaultFinishWhich};
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kStart))
      .WillRepeatedly(Return(default_branch_start));
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kFinish))
      .WillRepeatedly(Return(default_branch_finish));

  const maliput_ros_interfaces::msg::Lane dut = ToRosMessage(&lane);

  ASSERT_EQ(dut.id.id, kLaneId.string());
  ASSERT_EQ(dut.segment_id.id, kSegmentId.string());
  ASSERT_EQ(dut.index, kIndex);
  ASSERT_EQ(dut.left_lane.id, kLeftLaneId.string());
  ASSERT_EQ(dut.right_lane.id, kRightLaneId.string());
  ASSERT_EQ(dut.length, kLaneLength);
  ASSERT_EQ(dut.start_branch_point_id.id, kStartBranchPointId.string());
  ASSERT_EQ(dut.finish_branch_point_id.id, kFinishBranchPointId.string());
  ASSERT_EQ(dut.default_start_branch.lane_id.id, kDefaultStartLaneId.string());
  ASSERT_EQ(dut.default_start_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.default_finish_branch.lane_id.id, kDefaultFinishLaneId.string());
  ASSERT_EQ(dut.default_finish_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(LaneToRosMessageTest, WithoutDefaultStartBranch) {
  const std::optional<maliput::api::LaneEnd> default_branch_start{};
  const std::optional<maliput::api::LaneEnd> default_branch_finish =
      maliput::api::LaneEnd{&default_finish_lane, kDefaultFinishWhich};
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kStart))
      .WillRepeatedly(Return(default_branch_start));
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kFinish))
      .WillRepeatedly(Return(default_branch_finish));

  const maliput_ros_interfaces::msg::Lane dut = ToRosMessage(&lane);

  ASSERT_EQ(dut.id.id, kLaneId.string());
  ASSERT_EQ(dut.segment_id.id, kSegmentId.string());
  ASSERT_EQ(dut.index, kIndex);
  ASSERT_EQ(dut.left_lane.id, kLeftLaneId.string());
  ASSERT_EQ(dut.right_lane.id, kRightLaneId.string());
  ASSERT_EQ(dut.length, kLaneLength);
  ASSERT_EQ(dut.start_branch_point_id.id, kStartBranchPointId.string());
  ASSERT_EQ(dut.finish_branch_point_id.id, kFinishBranchPointId.string());
  ASSERT_EQ(dut.default_start_branch.lane_id.id, kNullLaneId);
  ASSERT_EQ(dut.default_finish_branch.lane_id.id, kDefaultFinishLaneId.string());
  ASSERT_EQ(dut.default_finish_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(LaneToRosMessageTest, WithoutDefaultFinishBranch) {
  const std::optional<maliput::api::LaneEnd> default_branch_start =
      maliput::api::LaneEnd{&default_start_lane, kDefaultStartWhich};
  const std::optional<maliput::api::LaneEnd> default_branch_finish{};
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kStart))
      .WillRepeatedly(Return(default_branch_start));
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kFinish))
      .WillRepeatedly(Return(default_branch_finish));

  const maliput_ros_interfaces::msg::Lane dut = ToRosMessage(&lane);

  ASSERT_EQ(dut.id.id, kLaneId.string());
  ASSERT_EQ(dut.segment_id.id, kSegmentId.string());
  ASSERT_EQ(dut.index, kIndex);
  ASSERT_EQ(dut.left_lane.id, kLeftLaneId.string());
  ASSERT_EQ(dut.right_lane.id, kRightLaneId.string());
  ASSERT_EQ(dut.length, kLaneLength);
  ASSERT_EQ(dut.start_branch_point_id.id, kStartBranchPointId.string());
  ASSERT_EQ(dut.finish_branch_point_id.id, kFinishBranchPointId.string());
  ASSERT_EQ(dut.default_start_branch.lane_id.id, kDefaultStartLaneId.string());
  ASSERT_EQ(dut.default_start_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(dut.default_finish_branch.lane_id.id, kNullLaneId);
}

TEST_F(LaneToRosMessageTest, NullLane) {
  const maliput_ros_interfaces::msg::Lane dut = ToRosMessage(static_cast<const maliput::api::Lane*>(nullptr));

  ASSERT_EQ(dut.id.id, kNullLaneId);
}

}  // namespace
}  // namespace test
}  // namespace maliput_ros_translation
