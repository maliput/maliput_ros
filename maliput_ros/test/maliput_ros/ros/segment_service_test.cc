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

// Test class to wrap the tests of /segment service call.
class SegmentByIdServiceCallTest : public MaliputQueryNodeAfterConfigurationTest {
 public:
  void SetUp() override {
    MaliputQueryNodeAfterConfigurationTest::SetUp();
    AddNodeToExecutorAndSpin(dut_);
    TransitionToConfigureFromUnconfigured();
    TransitionToActiveFromConfigured();
  }
};

TEST_F(SegmentByIdServiceCallTest, ValidResquestAndResponse) {
  static constexpr int kSize{2};
  const maliput::api::LaneId kLaneIdA{"lane_id_a"};
  const maliput::api::LaneId kLaneIdB{"lane_id_b"};
  const maliput::api::JunctionId kJunctionId{"junction_id"};
  const maliput::api::SegmentId kSegmentId{"segment_id"};
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  JunctionMock junction;
  EXPECT_CALL(junction, do_id()).WillRepeatedly(Return(kJunctionId));
  SegmentMock segment;
  EXPECT_CALL(segment, do_id()).WillRepeatedly(Return(kSegmentId));
  EXPECT_CALL(segment, do_junction()).WillRepeatedly(Return(&junction));
  EXPECT_CALL(segment, do_num_lanes()).WillRepeatedly(Return(kSize));
  EXPECT_CALL(segment, do_lane(0)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(segment, do_lane(1)).WillRepeatedly(Return(&lane_b));
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetSegment(kSegmentId)).WillRepeatedly(Return(&segment));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Segment::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kSegmentId);

  auto response =
      MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Segment>(kSegmentServiceName, kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_EQ(response->segment.id.id, kSegmentId.string());
  ASSERT_EQ(response->segment.junction_id.id, kJunctionId.string());
  ASSERT_EQ(response->segment.lane_ids.size(), static_cast<size_t>(kSize));
  ASSERT_EQ(response->segment.lane_ids[0].id, kLaneIdA.string());
  ASSERT_EQ(response->segment.lane_ids[1].id, kLaneIdB.string());
}

TEST_F(SegmentByIdServiceCallTest, InvalidIdReturnsEmptyResponse) {
  const maliput::api::SegmentId kSegmentId{"invalid_id"};
  IdIndexMock id_index;
  EXPECT_CALL(id_index, DoGetSegment(kSegmentId)).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(*(road_network_ptrs_.road_geometry), DoById()).WillRepeatedly(ReturnRef(id_index));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Segment::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kSegmentId);

  auto response =
      MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Segment>(kSegmentServiceName, kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->segment.id.id.empty());
}

TEST_F(SegmentByIdServiceCallTest, EmptyIdReturnsEmptyResponse) {
  auto request = std::make_shared<maliput_ros_interfaces::srv::Segment::Request>();
  request->id.id = "";

  auto response =
      MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Segment>(kSegmentServiceName, kTimeoutServiceCall, request);

  ASSERT_NE(response, nullptr);
  ASSERT_TRUE(response->segment.id.id.empty());
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
