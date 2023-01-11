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

// Test class to wrap the tests of /branch_point service call.
class BranchPointByIdServiceCallTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(BranchPointByIdServiceCallTest, ValidResquestAndResponse) {
  const maliput::api::LaneId kLaneIdA{"lane_id_a"};
  const maliput::api::LaneId kLaneIdB{"lane_id_b"};
  const maliput::api::LaneEnd::Which kWhichA{maliput::api::LaneEnd::Which::kStart};
  const maliput::api::LaneEnd::Which kWhichB{maliput::api::LaneEnd::Which::kFinish};
  const maliput::api::RoadGeometryId kRoadGeometryId{"kRoadGeometryId"};
  const maliput::api::BranchPointId kBranchPointId{"branch_point_id"};
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  const maliput::api::LaneEnd lane_end_a(&lane_a, kWhichA);
  const maliput::api::LaneEnd lane_end_b(&lane_b, kWhichB);
  LaneEndSetMock lane_end_set_a;
  EXPECT_CALL(lane_end_set_a, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(lane_end_set_a, do_get(0)).WillRepeatedly(ReturnRef(lane_end_a));
  LaneEndSetMock lane_end_set_b;
  EXPECT_CALL(lane_end_set_b, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(lane_end_set_b, do_get(0)).WillRepeatedly(ReturnRef(lane_end_b));
  RoadGeometryMock road_geometry;
  EXPECT_CALL(road_geometry, do_id()).WillRepeatedly(Return(kRoadGeometryId));
  BranchPointMock branch_point;
  EXPECT_CALL(branch_point, do_id()).WillRepeatedly(Return(kBranchPointId));
  EXPECT_CALL(branch_point, do_road_geometry()).WillRepeatedly(Return(&road_geometry));
  EXPECT_CALL(branch_point, DoGetASide()).WillRepeatedly(Return(&lane_end_set_a));
  EXPECT_CALL(branch_point, DoGetBSide()).WillRepeatedly(Return(&lane_end_set_b));

  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetBranchPoint(kBranchPointId)).WillRepeatedly(Return(&branch_point));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  auto request = std::make_shared<maliput_ros_interfaces::srv::BranchPoint::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kBranchPointId);

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::BranchPoint>(kBranchPointServiceName,
                                                                                    kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->branch_point.id.id, kBranchPointId.string());
  ASSERT_EQ(response->branch_point.road_geometry_id.id, kRoadGeometryId.string());
  ASSERT_EQ(response->branch_point.a_side.lane_ends.size(), static_cast<size_t>(1));
  ASSERT_EQ(response->branch_point.a_side.lane_ends[0].lane_id.id, kLaneIdA.string());
  ASSERT_EQ(response->branch_point.a_side.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHSTART);
  ASSERT_EQ(response->branch_point.b_side.lane_ends.size(), static_cast<size_t>(1));
  ASSERT_EQ(response->branch_point.b_side.lane_ends[0].lane_id.id, kLaneIdB.string());
  ASSERT_EQ(response->branch_point.b_side.lane_ends[0].end, maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH);
}

TEST_F(BranchPointByIdServiceCallTest, InvalidIdReturnsEmptyResponse) {
  const maliput::api::BranchPointId kBranchPointId{"invalid_id"};
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetBranchPoint(kBranchPointId)).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  auto request = std::make_shared<maliput_ros_interfaces::srv::BranchPoint::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kBranchPointId);

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::BranchPoint>(kBranchPointServiceName,
                                                                                    kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->branch_point.id.id.empty());
}

TEST_F(BranchPointByIdServiceCallTest, EmptyIdReturnsEmptyResponse) {
  auto request = std::make_shared<maliput_ros_interfaces::srv::BranchPoint::Request>();
  request->id.id = "";

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::BranchPoint>(kBranchPointServiceName,
                                                                                    kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->branch_point.id.id.empty());
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
