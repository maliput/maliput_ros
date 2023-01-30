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
#include "maliput_ros/ros/maliput_query_node.h"

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <maliput/common/filesystem.h>
#include <rcl_lifecycle/rcl_lifecycle.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "maliput_ros/ros/maliput_mock.h"
#include "maliput_ros/ros/maliput_query_node_test_utils.h"
#include "maliput_ros_translation/convert.h"

namespace maliput_ros {
namespace ros {
namespace test {
namespace {

using ::testing::Return;
using ::testing::ReturnRef;

// Makes sure the name and the namespace are properly passed to the parent rclcpp_lifecycle::LifecycleNode class.
TEST_F(MaliputQueryNodeTest, ConstructorArguments) {
  auto dut = std::make_shared<MaliputQueryNode>(kNodeName, kNodeNamespace);

  ASSERT_EQ(std::string{kNodeName}, dut->get_name());
  ASSERT_EQ(std::string{kNodeNamespace}, dut->get_namespace());
}

// Makes sure that after construction, the node adds the "yaml_configuration_path" parameter.
// "use_sim_time" has been added by the parent node.
TEST_F(MaliputQueryNodeTest, DefineYamlConfigurationPathParameter) {
  const std::set<std::string> expected_parameter_names_set = {
      kYamlConfigurationPathParameterName,  // Defined by MaliputQueryNode
      "use_sim_time",                       // defined by LifecycleNode
  };

  auto dut = std::make_shared<MaliputQueryNode>(kNodeName, kNodeNamespace);
  const std::vector<std::string> dut_parameter_names_vector = dut->list_parameters({}, 0u).names;
  const std::set<std::string> dut_parameter_names_set{dut_parameter_names_vector.begin(),
                                                      dut_parameter_names_vector.end()};

  ASSERT_EQ(expected_parameter_names_set, dut_parameter_names_set);
}

// Makes sure the "yaml_configuration_path" parameter can be set.
TEST_F(MaliputQueryNodeTest, CanSetYamlConfigurationPathParameter) {
  static constexpr const char* kParameterValue = "a_path";
  const rclcpp::NodeOptions kNodeOptions = rclcpp::NodeOptions().append_parameter_override(
      kYamlConfigurationPathParameterName, rclcpp::ParameterValue(kParameterValue));

  auto dut = std::make_shared<MaliputQueryNode>(kNodeName, kNodeNamespace, kNodeOptions);

  ASSERT_EQ(kParameterValue, dut->get_parameter(kYamlConfigurationPathParameterName).as_string());
}

// Evaluates that the transition to the configure state tries to load the YAML configuration.
// Upon a bad file, this transition fails.
TEST_F(MaliputQueryNodeTest, WrongYamlConfigurationPathMakesTheConfigurationStageToFail) {
  static constexpr const char* kParameterValue = "wrong_file_path";
  const rclcpp::NodeOptions kNodeOptions = rclcpp::NodeOptions().append_parameter_override(
      kYamlConfigurationPathParameterName, rclcpp::ParameterValue(kParameterValue));

  auto ret = kCallbackFailure;

  auto dut = std::make_shared<MaliputQueryNode>(kNodeName, kNodeNamespace, kNodeOptions);
  ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, dut->get_current_state().id());
  dut->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE), ret);
  ASSERT_EQ(kCallbackFailure, ret);
}

// Makes sure the transition from UNCONFIGURED via CONFIGURE is successful.
TEST_F(MaliputQueryNodeAfterConfigurationTest, CorrectYamlConfigurationPathMakesTheConfigurationStageToPass) {
  TransitionToConfigureFromUnconfigured();
}

// Makes sure the services are advertised when passing the configuration stage.
TEST_F(MaliputQueryNodeAfterConfigurationTest, ConfigureStateAdvertisesServices) {
  TransitionToConfigureFromUnconfigured();
  ASSERT_TRUE(WaitForService(dut_, kRoadGeometryServiceName, kTimeout, kSleepPeriod));

  auto service_names_and_types = dut_->get_service_names_and_types();

  ASSERT_STREQ(service_names_and_types[kRoadGeometryServiceName][0].c_str(), kRoadGeometryServiceType);
  ASSERT_STREQ(service_names_and_types[kBranchPointServiceName][0].c_str(), kBranchPointServiceType);
  ASSERT_STREQ(service_names_and_types[kJunctionServiceName][0].c_str(), kJunctionServiceType);
  ASSERT_STREQ(service_names_and_types[kLaneServiceName][0].c_str(), kLaneServiceType);
  ASSERT_STREQ(service_names_and_types[kLaneBoundariesServiceName][0].c_str(), kLaneBoundariesServiceType);
  ASSERT_STREQ(service_names_and_types[kSegmentServiceName][0].c_str(), kSegmentServiceType);
  ASSERT_STREQ(service_names_and_types[kToRoadPositionServiceName][0].c_str(), kToRoadPositionServiceType);
  ASSERT_STREQ(service_names_and_types[kFindRoadPositionsServiceName][0].c_str(), kFindRoadPositionsServiceType);
  ASSERT_STREQ(service_names_and_types[kToInertialPoseServiceName][0].c_str(), kToInertialPoseServiceType);
  ASSERT_STREQ(service_names_and_types[kEvalMotionDerivativesServiceName][0].c_str(),
               kEvalMotionDerivativesServiceType);
  ASSERT_STREQ(service_names_and_types[kDeriveLaneSRoutesServiceName][0].c_str(), kDeriveLaneSRoutesServiceType);
  ASSERT_STREQ(service_names_and_types[kSampleLaneSRouteServiceName][0].c_str(), kSampleLaneSRouteServiceType);
}

// Makes sure services don't process the request when the node is not ACTIVE.
TEST_F(MaliputQueryNodeAfterConfigurationTest, CallingServiceBeforeActiveYieldsToFailure) {
  AddNodeToExecutorAndSpin(dut_);
  TransitionToConfigureFromUnconfigured();
  {
    ASSERT_TRUE(WaitForService(dut_, kRoadGeometryServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::RoadGeometry::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::RoadGeometry>(kRoadGeometryServiceName,
                                                                                       kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->road_geometry.id.id.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kBranchPointServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::BranchPoint::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::BranchPoint>(kBranchPointServiceName,
                                                                                      kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->branch_point.id.id.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kJunctionServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::Junction::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Junction>(kJunctionServiceName,
                                                                                   kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->junction.id.id.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kSegmentServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::Segment::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Segment>(kSegmentServiceName,
                                                                                  kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->segment.id.id.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kLaneServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::Lane::Request>();
    auto response =
        MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::Lane>(kLaneServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->lane.id.id.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kLaneBoundariesServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::LaneBoundaries::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::LaneBoundaries>(kLaneBoundariesServiceName,
                                                                                         kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kToRoadPositionServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::ToRoadPosition::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::ToRoadPosition>(kToRoadPositionServiceName,
                                                                                         kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->road_position_result.road_position.lane_id.id.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kFindRoadPositionsServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::FindRoadPositions::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::FindRoadPositions>(
        kFindRoadPositionsServiceName, kTimeoutServiceCall, request);
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->road_position_results.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kToInertialPoseServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::ToInertialPose::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::ToInertialPose>(kToInertialPoseServiceName,
                                                                                         kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kEvalMotionDerivativesServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::EvalMotionDerivatives::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::EvalMotionDerivatives>(
        kEvalMotionDerivativesServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_EQ(response->lane_derivatives.s, 0.);
    ASSERT_EQ(response->lane_derivatives.r, 0.);
    ASSERT_EQ(response->lane_derivatives.h, 0.);
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kDeriveLaneSRoutesServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::DeriveLaneSRoutes::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::DeriveLaneSRoutes>(
        kDeriveLaneSRoutesServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->lane_s_routes.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kSampleLaneSRouteServiceName, kTimeout, kSleepPeriod));

    auto request = std::make_shared<maliput_ros_interfaces::srv::SampleLaneSRoute::Request>();
    auto response = MakeAsyncRequestAndWait<maliput_ros_interfaces::srv::SampleLaneSRoute>(
        kSampleLaneSRouteServiceName, kTimeoutServiceCall, request);

    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->waypoints.empty());
  }
}

// Makes sure the node can transtion to the ACTIVE state.
TEST_F(MaliputQueryNodeAfterConfigurationTest, TransitionToActiveIsSuccessful) {
  TransitionToConfigureFromUnconfigured();
  TransitionToActiveFromConfigured();
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
