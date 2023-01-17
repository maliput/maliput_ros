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

// Test class to wrap the tests of /to_road_position and /find_road_positions services.
class RoadPostionServicesCallTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(RoadPostionServicesCallTest, ToRoadPositionValidRequest) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LanePosition kLanePosition(1., 2., 3.);
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  const maliput::api::InertialPosition kNearestPosition(7., 8., 9.);
  constexpr double kDistance{10.};
  const maliput::api::RoadPositionResult kRoadPositionResult{kRoadPosition, kNearestPosition, kDistance};
  const maliput::api::InertialPosition kInertialPosition(4., 5., 6.);
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoToRoadPosition(kInertialPosition, ::testing::_))
      .WillRepeatedly(Return(kRoadPositionResult));
  auto request = std::make_shared<maliput_ros_interfaces::srv::ToRoadPosition::Request>();
  request->inertial_position = maliput_ros_translation::ToRosMessage(kInertialPosition);

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::ToRoadPosition>(kToRoadPositionServiceName,
                                                                                       kTimeoutServiceCall, request);

  ASSERT_EQ(response->road_position_result.road_position.lane_id.id, kLaneId.string());
  ASSERT_EQ(response->road_position_result.road_position.pos.s, kLanePosition.s());
  ASSERT_EQ(response->road_position_result.road_position.pos.r, kLanePosition.r());
  ASSERT_EQ(response->road_position_result.road_position.pos.h, kLanePosition.h());
  ASSERT_EQ(response->road_position_result.nearest_position.x, kNearestPosition.x());
  ASSERT_EQ(response->road_position_result.nearest_position.y, kNearestPosition.y());
  ASSERT_EQ(response->road_position_result.nearest_position.z, kNearestPosition.z());
  ASSERT_EQ(response->road_position_result.distance, kDistance);
}

TEST_F(RoadPostionServicesCallTest, FindRoadPositionsValidRequest) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LanePosition kLanePosition(1., 2., 3.);
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  const maliput::api::InertialPosition kNearestPosition(7., 8., 9.);
  constexpr double kDistance{10.};
  const maliput::api::RoadPositionResult kRoadPositionResult{kRoadPosition, kNearestPosition, kDistance};
  const maliput::api::InertialPosition kInertialPosition(4., 5., 6.);
  constexpr double kRadius{12.};
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoFindRoadPositions(kInertialPosition, kRadius))
      .WillRepeatedly(Return(std::vector<maliput::api::RoadPositionResult>{kRoadPositionResult}));
  auto request = std::make_shared<maliput_ros_interfaces::srv::FindRoadPositions::Request>();
  request->inertial_position = maliput_ros_translation::ToRosMessage(kInertialPosition);
  request->radius = kRadius;

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::FindRoadPositions>(kFindRoadPositionsServiceName,
                                                                                          kTimeoutServiceCall, request);

  ASSERT_EQ(response->road_position_results.size(), 1u);
  ASSERT_EQ(response->road_position_results[0].road_position.lane_id.id, kLaneId.string());
  ASSERT_EQ(response->road_position_results[0].road_position.pos.s, kLanePosition.s());
  ASSERT_EQ(response->road_position_results[0].road_position.pos.r, kLanePosition.r());
  ASSERT_EQ(response->road_position_results[0].road_position.pos.h, kLanePosition.h());
  ASSERT_EQ(response->road_position_results[0].nearest_position.x, kNearestPosition.x());
  ASSERT_EQ(response->road_position_results[0].nearest_position.y, kNearestPosition.y());
  ASSERT_EQ(response->road_position_results[0].nearest_position.z, kNearestPosition.z());
  ASSERT_EQ(response->road_position_results[0].distance, kDistance);
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros