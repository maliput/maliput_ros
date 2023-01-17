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
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "maliput_ros/ros/maliput_mock.h"
#include "maliput_ros/ros/maliput_query_node.h"
#include "maliput_ros/ros/maliput_query_node_test_utils.h"
#include "maliput_ros_translation/convert.h"

namespace maliput_ros {
namespace ros {
namespace test {
namespace {

using ::testing::Return;
using ::testing::ReturnRef;

// Test class to wrap the tests of /lane service call.
class LaneByIdServiceCallTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(LaneByIdServiceCallTest, ValidResquestAndResponse) {
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
  SegmentMock segment;
  EXPECT_CALL(segment, do_id()).WillRepeatedly(Return(kSegmentId));
  LaneMock left_lane;
  EXPECT_CALL(left_lane, do_id()).WillRepeatedly(Return(kLeftLaneId));
  LaneMock right_lane;
  EXPECT_CALL(right_lane, do_id()).WillRepeatedly(Return(kRightLaneId));
  LaneMock default_start_lane;
  EXPECT_CALL(default_start_lane, do_id()).WillRepeatedly(Return(kDefaultStartLaneId));
  LaneMock default_finish_lane;
  EXPECT_CALL(default_finish_lane, do_id()).WillRepeatedly(Return(kDefaultFinishLaneId));
  BranchPointMock start_branch_point;
  EXPECT_CALL(start_branch_point, do_id()).WillRepeatedly(Return(kStartBranchPointId));
  BranchPointMock finish_branch_point;
  EXPECT_CALL(finish_branch_point, do_id()).WillRepeatedly(Return(kFinishBranchPointId));
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  EXPECT_CALL(lane, do_segment()).WillRepeatedly(Return(&segment));
  EXPECT_CALL(lane, do_index()).WillRepeatedly(Return(kIndex));
  EXPECT_CALL(lane, do_to_left()).WillRepeatedly(Return(&left_lane));
  EXPECT_CALL(lane, do_to_right()).WillRepeatedly(Return(&right_lane));
  EXPECT_CALL(lane, do_length()).WillRepeatedly(Return(kLaneLength));
  EXPECT_CALL(lane, DoGetBranchPoint(maliput::api::LaneEnd::Which::kStart)).WillRepeatedly(Return(&start_branch_point));
  EXPECT_CALL(lane, DoGetBranchPoint(maliput::api::LaneEnd::Which::kFinish))
      .WillRepeatedly(Return(&finish_branch_point));
  const std::optional<maliput::api::LaneEnd> default_branch_start =
      maliput::api::LaneEnd{&default_start_lane, kDefaultStartWhich};
  const std::optional<maliput::api::LaneEnd> default_branch_finish =
      maliput::api::LaneEnd{&default_finish_lane, kDefaultFinishWhich};
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kStart))
      .WillRepeatedly(Return(default_branch_start));
  EXPECT_CALL(lane, DoGetDefaultBranch(maliput::api::LaneEnd::Which::kFinish))
      .WillRepeatedly(Return(default_branch_finish));
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(&lane));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Lane::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kLaneId);

  auto response =
      MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Lane>(kLaneServiceName, kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->lane.id.id, kLaneId.string());
  ASSERT_EQ(response->lane.segment_id.id, kSegmentId.string());
  ASSERT_EQ(response->lane.index, kIndex);
  ASSERT_EQ(response->lane.left_lane.id, kLeftLaneId.string());
  ASSERT_EQ(response->lane.right_lane.id, kRightLaneId.string());
  ASSERT_EQ(response->lane.length, kLaneLength);
  ASSERT_EQ(response->lane.start_branch_point_id.id, kStartBranchPointId.string());
  ASSERT_EQ(response->lane.finish_branch_point_id.id, kFinishBranchPointId.string());
  ASSERT_EQ(response->lane.default_start_branch.lane_id.id, kDefaultStartLaneId.string());
  ASSERT_EQ(response->lane.default_start_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(response->lane.default_finish_branch.lane_id.id, kDefaultFinishLaneId.string());
  ASSERT_EQ(response->lane.default_finish_branch.end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(LaneByIdServiceCallTest, InvalidIdReturnsEmptyResponse) {
  const maliput::api::LaneId kLaneId{"invalid_id"};
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Lane::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kLaneId);

  auto response =
      MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Lane>(kLaneServiceName, kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->lane.id.id.empty());
}

TEST_F(LaneByIdServiceCallTest, EmptyIdReturnsEmptyResponse) {
  auto request = std::make_shared<maliput_ros_interfaces::srv::Lane::Request>();
  request->id.id = "";

  auto response =
      MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Lane>(kLaneServiceName, kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->lane.id.id.empty());
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
