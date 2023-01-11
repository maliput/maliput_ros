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

// Test class to wrap the tests of /junction service call.
class JunctionByIdServiceCallTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(JunctionByIdServiceCallTest, ValidResquestAndResponse) {
  static constexpr int kSize{2};
  const maliput::api::SegmentId kSgmentIdA{"segment_id_a"};
  const maliput::api::SegmentId kSgmentIdB{"segment_id_b"};
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  SegmentMock segment_a;
  EXPECT_CALL(segment_a, do_id()).WillRepeatedly(Return(kSgmentIdA));
  SegmentMock segment_b;
  EXPECT_CALL(segment_b, do_id()).WillRepeatedly(Return(kSgmentIdB));
  JunctionMock junction;
  EXPECT_CALL(junction, do_id()).WillRepeatedly(Return(kJunctionId));
  EXPECT_CALL(junction, do_road_geometry()).WillRepeatedly(Return(road_network_ptrs_.road_geometry));
  EXPECT_CALL(junction, do_num_segments()).WillRepeatedly(Return(kSize));
  EXPECT_CALL(junction, do_segment(0)).WillRepeatedly(Return(&segment_a));
  EXPECT_CALL(junction, do_segment(1)).WillRepeatedly(Return(&segment_b));
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetJunction(kJunctionId)).WillRepeatedly(Return(&junction));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), do_id()).WillRepeatedly(Return(kRoadGeometryId));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Junction::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kJunctionId);

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Junction>(kJunctionServiceName,
                                                                                 kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->junction.id.id, kJunctionId.string());
  ASSERT_EQ(response->junction.road_geometry_id.id, kRoadGeometryId.string());
  ASSERT_EQ(response->junction.segment_ids.size(), static_cast<size_t>(kSize));
  ASSERT_EQ(response->junction.segment_ids[0].id, kSgmentIdA.string());
  ASSERT_EQ(response->junction.segment_ids[1].id, kSgmentIdB.string());
}

TEST_F(JunctionByIdServiceCallTest, InvalidIdReturnsEmptyResponse) {
  const maliput::api::RoadGeometryId kRoadGeometryId{"road_geometry_id"};
  const maliput::api::JunctionId kJunctionId{"invalid_id"};
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetJunction(kJunctionId)).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Junction::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kJunctionId);

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Junction>(kJunctionServiceName,
                                                                                 kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->junction.id.id.empty());
}

TEST_F(JunctionByIdServiceCallTest, EmptyIdReturnsEmptyResponse) {
  auto request = std::make_shared<maliput_ros_interfaces::srv::Junction::Request>();
  request->id.id = "";

  auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Junction>(kJunctionServiceName,
                                                                                 kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->junction.id.id.empty());
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
