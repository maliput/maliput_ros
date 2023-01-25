// BSD 3-Clause License
//
// Copyright (c) 2023, Woven Planet. All rights reserved.
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
#include <maliput/api/lane_data.h>
#include <maliput/math/quaternion.h>
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

// Test class to wrap the tests of /derive_lane_s_routes service call.
class DeriveLaneSRoutesServiceTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  static constexpr double kMaxLengthM{1e6};
  static constexpr double kInvalidMaxLengthM{-1};
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::LanePosition kStartLanePosition{1., 0., 0.};
  const maliput::api::LanePosition kEndLanePosition{2., 0., 0.};
  const maliput::api::RoadPosition kInvalidRoadPosition;

  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(DeriveLaneSRoutesServiceTest, ValidRequestAndResponse) {
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::RoadPosition kStartRoadPosition{&lane, kStartLanePosition};
  const maliput::api::RoadPosition kEndRoadPosition{&lane, kEndLanePosition};
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(&lane));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  auto request = std::make_shared<maliput_ros_interfaces::srv::DeriveLaneSRoutes::Request>();
  request->start = maliput_ros_translation::ToRosMessage(kStartRoadPosition);
  request->end = maliput_ros_translation::ToRosMessage(kEndRoadPosition);
  request->max_length_m = kMaxLengthM;

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::DeriveLaneSRoutes>(kDeriveLaneSRoutesServiceName,
                                                                                          kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->lane_s_routes.size(), 1u);
  ASSERT_EQ(response->lane_s_routes[0].ranges.size(), 1u);
  ASSERT_EQ(response->lane_s_routes[0].ranges[0].lane_id.id, kLaneId.string());
  ASSERT_EQ(response->lane_s_routes[0].ranges[0].s_range.s0, kStartLanePosition.s());
  ASSERT_EQ(response->lane_s_routes[0].ranges[0].s_range.s1, kEndLanePosition.s());
}

TEST_F(DeriveLaneSRoutesServiceTest, InvalidRequest) {
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::RoadPosition kStartRoadPosition{&lane, kStartLanePosition};
  const maliput::api::RoadPosition kEndRoadPosition{&lane, kEndLanePosition};
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(&lane));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));

  {
    auto request = std::make_shared<maliput_ros_interfaces::srv::DeriveLaneSRoutes::Request>();
    request->start = maliput_ros_translation::ToRosMessage(kInvalidRoadPosition);
    request->end = maliput_ros_translation::ToRosMessage(kEndRoadPosition);
    request->max_length_m = kMaxLengthM;

    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::DeriveLaneSRoutes>(
        kDeriveLaneSRoutesServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->lane_s_routes.empty());
  }
  {
    auto request = std::make_shared<maliput_ros_interfaces::srv::DeriveLaneSRoutes::Request>();
    request->start = maliput_ros_translation::ToRosMessage(kStartRoadPosition);
    request->end = maliput_ros_translation::ToRosMessage(kInvalidRoadPosition);
    request->max_length_m = kMaxLengthM;

    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::DeriveLaneSRoutes>(
        kDeriveLaneSRoutesServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->lane_s_routes.empty());
  }
  {
    auto request = std::make_shared<maliput_ros_interfaces::srv::DeriveLaneSRoutes::Request>();
    request->start = maliput_ros_translation::ToRosMessage(kStartRoadPosition);
    request->end = maliput_ros_translation::ToRosMessage(kStartRoadPosition);
    request->max_length_m = kInvalidMaxLengthM;

    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::DeriveLaneSRoutes>(
        kDeriveLaneSRoutesServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->lane_s_routes.empty());
  }
}

// Test class to wrap the tests of /sample_lane_s_route service call.
class SampleLaneSRouteServiceTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  const maliput::api::LaneId kLaneId{"lane_id"};
  const maliput::api::LaneSRange kLaneSRange{kLaneId, maliput::api::SRange{10., 20.}};
  const maliput::api::LaneSRoute kLaneSRoute{{kLaneSRange}};
  const maliput::api::InertialPosition kInertialPosition1{10., 5., 1.};
  const maliput::api::InertialPosition kInertialPosition2{15., 5., 1.};
  const maliput::api::InertialPosition kInertialPosition3{20., 5., 1.};
  const std::vector<maliput::api::InertialPosition> kExpectedResult{kInertialPosition1, kInertialPosition2,
                                                                    kInertialPosition3};
  static constexpr double kPathLengthSamplingRate{5.};

  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(SampleLaneSRouteServiceTest, ValidRequestAndResponse) {
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoSampleAheadWaypoints(::testing::_, kPathLengthSamplingRate))
      .WillRepeatedly(Return(kExpectedResult));
  auto request = std::make_shared<maliput_ros_interfaces::srv::SampleLaneSRoute::Request>();
  request->lane_s_route = maliput_ros_translation::ToRosMessage(kLaneSRoute);
  request->path_length_sampling_rate = kPathLengthSamplingRate;

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::SampleLaneSRoute>(kSampleLaneSRouteServiceName,
                                                                                         kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->waypoints.size(), 3u);
  ASSERT_EQ(response->waypoints[0].x, kInertialPosition1.x());
  ASSERT_EQ(response->waypoints[0].y, kInertialPosition1.y());
  ASSERT_EQ(response->waypoints[0].z, kInertialPosition1.z());
  ASSERT_EQ(response->waypoints[1].x, kInertialPosition2.x());
  ASSERT_EQ(response->waypoints[1].y, kInertialPosition2.y());
  ASSERT_EQ(response->waypoints[1].z, kInertialPosition2.z());
  ASSERT_EQ(response->waypoints[2].x, kInertialPosition3.x());
  ASSERT_EQ(response->waypoints[2].y, kInertialPosition3.y());
  ASSERT_EQ(response->waypoints[2].z, kInertialPosition3.z());
}

TEST_F(SampleLaneSRouteServiceTest, InvalidRequest) {
  // Invalid LaneSRoute.
  {
    auto request = std::make_shared<maliput_ros_interfaces::srv::SampleLaneSRoute::Request>();
    request->path_length_sampling_rate = kPathLengthSamplingRate;

    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::SampleLaneSRoute>(
        kSampleLaneSRouteServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->waypoints.empty());
  }
  // Invalid sampling rate.
  {
    static constexpr double kInvalidPathLengthSamplingRate{-1.};
    auto request = std::make_shared<maliput_ros_interfaces::srv::SampleLaneSRoute::Request>();
    request->lane_s_route = maliput_ros_translation::ToRosMessage(kLaneSRoute);
    request->path_length_sampling_rate = kInvalidPathLengthSamplingRate;

    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::SampleLaneSRoute>(
        kSampleLaneSRouteServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->waypoints.empty());
  }
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
