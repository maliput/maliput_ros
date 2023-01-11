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

// Test class used to hold the configuration of the RoadGeometryMock and validate the result of the service call.
class RoadGeometryServiceCallTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  static constexpr int kSizeJunctions{0};
  static constexpr int kSizeBranchPoints{0};
  static constexpr double kLinearTolerance{1e-12};
  static constexpr double kAngularTolerance{5e-12};
  static constexpr double kScaleLength{1.};
  const maliput::math::Vector3 kInertialToBackendFrameTranslation{1., 2., 3.};
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};

  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(RoadGeometryServiceCallTest, RoadGeometryRequestInActiveIsSuccessful) {
  // Note: there is no point in populating Junctions and BranchPoints, it is already covered
  // in maliput_ros_translation package.
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), do_id()).WillRepeatedly(Return(kRoadGeometryId));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), do_angular_tolerance()).WillRepeatedly(Return(kAngularTolerance));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), do_scale_length()).WillRepeatedly(Return(kScaleLength));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), do_inertial_to_backend_frame_translation())
      .WillRepeatedly(Return(kInertialToBackendFrameTranslation));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), do_num_junctions()).WillRepeatedly(Return(kSizeJunctions));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), do_num_branch_points()).WillRepeatedly(Return(kSizeBranchPoints));
  auto request = std::make_shared<maliput_ros_interfaces::srv::RoadGeometry::Request>();

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::RoadGeometry>(kRoadGeometryServiceName,
                                                                                     kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->road_geometry.id.id, kRoadGeometryId.string());
  ASSERT_EQ(response->road_geometry.linear_tolerance, kLinearTolerance);
  ASSERT_EQ(response->road_geometry.angular_tolerance, kAngularTolerance);
  ASSERT_EQ(response->road_geometry.scale_length, kScaleLength);
  ASSERT_EQ(response->road_geometry.inertial_to_backend_frame_translation.x, kInertialToBackendFrameTranslation.x());
  ASSERT_EQ(response->road_geometry.inertial_to_backend_frame_translation.y, kInertialToBackendFrameTranslation.y());
  ASSERT_EQ(response->road_geometry.inertial_to_backend_frame_translation.z, kInertialToBackendFrameTranslation.z());
  ASSERT_EQ(response->road_geometry.junction_ids.size(), static_cast<size_t>(kSizeJunctions));
  ASSERT_EQ(response->road_geometry.branch_point_ids.size(), static_cast<size_t>(kSizeJunctions));
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
