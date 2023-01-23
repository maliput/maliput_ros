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

// Test class to wrap the tests of /to_inertial_pose and /eval_motion_derivatives service calls.
class LaneGeometryServicesTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(LaneGeometryServicesTest, ToInertialPoseValidRequestAndResponse) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LanePosition kLanePosition{1., 2., 3.};
  const maliput::api::InertialPosition kInertialPosition{4., 5., 6.};
  EXPECT_CALL(lane, DoToInertialPosition(::testing::_)).WillRepeatedly(Return(kInertialPosition));
  // A quaternion that rotates x->y, y->z, z->x...
  const maliput::math::Quaternion kQuaternion =
      maliput::math::Quaternion(M_PI * 2. / 3., maliput::math::Vector3(1.0, 1.0, 1.0).normalized());
  const maliput::api::Rotation kRotation = maliput::api::Rotation::FromQuat(kQuaternion);
  EXPECT_CALL(lane, DoGetOrientation(::testing::_)).WillRepeatedly(Return(kRotation));
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(&lane));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  auto request = std::make_shared<maliput_ros_interfaces::srv::ToInertialPose::Request>();
  request->road_position = maliput_ros_translation::ToRosMessage(kRoadPosition);

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::ToInertialPose>(kToInertialPoseServiceName,
                                                                                       kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->position.x, kInertialPosition.x());
  ASSERT_EQ(response->position.y, kInertialPosition.y());
  ASSERT_EQ(response->position.z, kInertialPosition.z());
  ASSERT_EQ(response->orientation.x, kQuaternion.x());
  ASSERT_EQ(response->orientation.y, kQuaternion.y());
  ASSERT_EQ(response->orientation.z, kQuaternion.z());
  ASSERT_EQ(response->orientation.w, kQuaternion.w());
}

TEST_F(LaneGeometryServicesTest, ToInertialPoseInvalidRoadPosition) {
  auto request = std::make_shared<maliput_ros_interfaces::srv::ToInertialPose::Request>();
  request->road_position.lane_id.id = "";

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::ToInertialPose>(kToInertialPoseServiceName,
                                                                                       kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->position.x, 0.);
  ASSERT_EQ(response->position.y, 0.);
  ASSERT_EQ(response->position.z, 0.);
  ASSERT_EQ(response->orientation.x, 0.);
  ASSERT_EQ(response->orientation.y, 0.);
  ASSERT_EQ(response->orientation.z, 0.);
  ASSERT_EQ(response->orientation.w, 0.);
}

TEST_F(LaneGeometryServicesTest, EvalMotionDerivativesValidRoadPosition) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  const maliput::api::LanePosition kLaneDerivatives{4., 5., 6.};
  EXPECT_CALL(lane, DoEvalMotionDerivatives(::testing::_, ::testing::_)).WillRepeatedly(Return(kLaneDerivatives));
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(&lane));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  const maliput::api::LanePosition kLanePosition{1., 2., 3.};
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  const maliput::api::IsoLaneVelocity kVelocity{7., 8., 9.};
  auto request = std::make_shared<maliput_ros_interfaces::srv::EvalMotionDerivatives::Request>();
  request->road_position = maliput_ros_translation::ToRosMessage(kRoadPosition);
  request->velocity = maliput_ros_translation::ToRosMessage(kVelocity);

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::EvalMotionDerivatives>(
      kEvalMotionDerivativesServiceName, kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->lane_derivatives.s, kLaneDerivatives.s());
  ASSERT_EQ(response->lane_derivatives.r, kLaneDerivatives.r());
  ASSERT_EQ(response->lane_derivatives.h, kLaneDerivatives.h());
}

TEST_F(LaneGeometryServicesTest, EvalMotionDerivativesInvalidRoadPosition) {
  const maliput::api::IsoLaneVelocity kVelocity{1., 2., 3.};
  auto request = std::make_shared<maliput_ros_interfaces::srv::EvalMotionDerivatives::Request>();
  request->velocity = maliput_ros_translation::ToRosMessage(kVelocity);
  request->road_position.lane_id.id = "";

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::EvalMotionDerivatives>(
      kEvalMotionDerivativesServiceName, kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->lane_derivatives.s, 0.);
  ASSERT_EQ(response->lane_derivatives.r, 0.);
  ASSERT_EQ(response->lane_derivatives.h, 0.);
}

TEST_F(LaneGeometryServicesTest, LaneBoundariesValidRoadPosition) {
  const maliput::api::LaneId kLaneId{"lane_id"};
  LaneMock lane;
  EXPECT_CALL(lane, do_id()).WillRepeatedly(Return(kLaneId));
  static constexpr double kS = 1.;
  static constexpr double kR = 2.;
  const maliput::api::RBounds kLaneBounds{-4., 5.};
  EXPECT_CALL(lane, do_lane_bounds(kS)).WillRepeatedly(Return(kLaneBounds));
  const maliput::api::RBounds kSegmentBounds{-6., 7.};
  EXPECT_CALL(lane, do_segment_bounds(kS)).WillRepeatedly(Return(kSegmentBounds));
  const maliput::api::HBounds kElevationBounds{-8., 9.};
  EXPECT_CALL(lane, do_elevation_bounds(kS, kR)).WillRepeatedly(Return(kElevationBounds));
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetLane(kLaneId)).WillRepeatedly(Return(&lane));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  const maliput::api::LanePosition kLanePosition{kS, kR, 3.};
  const maliput::api::RoadPosition kRoadPosition{&lane, kLanePosition};
  auto request = std::make_shared<maliput_ros_interfaces::srv::LaneBoundaries::Request>();
  request->road_position = maliput_ros_translation::ToRosMessage(kRoadPosition);

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::LaneBoundaries>(kLaneBoundariesServiceName,
                                                                                       kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->lane_bounds.min, kLaneBounds.min());
  ASSERT_EQ(response->lane_bounds.max, kLaneBounds.max());
  ASSERT_EQ(response->segment_bounds.min, kSegmentBounds.min());
  ASSERT_EQ(response->segment_bounds.max, kSegmentBounds.max());
  ASSERT_EQ(response->elevation_bounds.min, kElevationBounds.min());
  ASSERT_EQ(response->elevation_bounds.max, kElevationBounds.max());
}

TEST_F(LaneGeometryServicesTest, LaneBoundariesWithInvalidRoadPosition) {
  auto request = std::make_shared<maliput_ros_interfaces::srv::LaneBoundaries::Request>();

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::LaneBoundaries>(kLaneBoundariesServiceName,
                                                                                       kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->lane_bounds.min, 0.);
  ASSERT_EQ(response->lane_bounds.max, 0.);
  ASSERT_EQ(response->segment_bounds.min, 0.);
  ASSERT_EQ(response->segment_bounds.max, 0.);
  ASSERT_EQ(response->elevation_bounds.min, 0.);
  ASSERT_EQ(response->elevation_bounds.max, 0.);
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
