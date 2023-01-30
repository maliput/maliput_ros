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
#pragma once

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
#include "maliput_ros/ros/maliput_plugin_config_test.h"

namespace maliput_ros {
namespace ros {
namespace test {

// @return A maliput::api::RoadNetwork populated with gmock versions of it, except maliput::api::rules::RuleRegistry.
inline std::unique_ptr<maliput::api::RoadNetwork> MakeRoadNetworkMock() {
  auto road_geometry = std::make_unique<RoadGeometryMock>();
  auto road_rulebook = std::make_unique<RoadRulebookMock>();
  auto traffic_light_book = std::make_unique<TrafficLightBookMock>();
  auto intersection_book = std::make_unique<IntersectionBookMock>();
  auto phase_ring_book = std::make_unique<PhaseRingBookMock>();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto right_of_way_rule_state_provider = std::make_unique<RightOfWayRuleStateProviderMock>();
#pragma GCC diagnostic pop
  auto phase_provider = std::make_unique<PhaseProviderMock>();
  auto discrete_value_rule_state_provider = std::make_unique<DiscreteValueRuleStateProviderMock>();
  auto range_value_rule_state_provider = std::make_unique<RangeValueRuleStateProviderMock>();

  return std::make_unique<maliput::api::RoadNetwork>(
      std::move(road_geometry), std::move(road_rulebook), std::move(traffic_light_book), std::move(intersection_book),
      std::move(phase_ring_book), std::move(right_of_way_rule_state_provider), std::move(phase_provider),
      std::make_unique<maliput::api::rules::RuleRegistry>(), std::move(discrete_value_rule_state_provider),
      std::move(range_value_rule_state_provider));
}

// Base test class for MaliputQueryNode.
class MaliputQueryNodeTest : public ::testing::Test {
 public:
  static constexpr const char* kNodeName = "my_name";
  static constexpr const char* kNodeNamespace = "/my_namespace";
  static constexpr const char* kYamlConfigurationPathParameterName = "yaml_configuration_path";
  static constexpr rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn kCallbackSuccess{
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS};
  static constexpr rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn kCallbackFailure{
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE};

  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override { executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(); }

  void TearDown() override {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  // Adds a node to a the executor_ and starts a thread to spin the executor.
  // Allows to process the service calls in this test while the futures are waited.
  //
  // @param[in] node The node to add to the executor_.
  void AddNodeToExecutorAndSpin(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {
    executor_->add_node(node->get_node_base_interface());
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  std::thread spin_thread_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// This test class is used when the node transitions successfully from the UNCONFIGURED to the ACTIVE stage passing by
// the CONFIGURATION transition. We provide a proper plugin configuration and each test must initialize the mocks
// according to the type of query.
class MaliputQueryNodeAfterConfigurationTest : public MaliputQueryNodeTest {
 public:
  static constexpr int kReplace{1};
  static constexpr const char* kEnvName = "MALIPUT_PLUGIN_PATH";
  static constexpr const char* kRoadNetowrkMockPluginPath = TEST_MALIPUT_PLUGIN_LIBDIR;
  static constexpr const char* kRoadGeometryServiceName = "/my_namespace/road_geometry";
  static constexpr const char* kRoadGeometryServiceType = "maliput_ros_interfaces/srv/RoadGeometry";
  static constexpr const char* kBranchPointServiceName = "/my_namespace/branch_point";
  static constexpr const char* kBranchPointServiceType = "maliput_ros_interfaces/srv/BranchPoint";
  static constexpr const char* kJunctionServiceName = "/my_namespace/junction";
  static constexpr const char* kJunctionServiceType = "maliput_ros_interfaces/srv/Junction";
  static constexpr const char* kLaneServiceName = "/my_namespace/lane";
  static constexpr const char* kLaneServiceType = "maliput_ros_interfaces/srv/Lane";
  static constexpr const char* kLaneBoundariesServiceName = "/my_namespace/lane_boundaries";
  static constexpr const char* kLaneBoundariesServiceType = "maliput_ros_interfaces/srv/LaneBoundaries";
  static constexpr const char* kSegmentServiceName = "/my_namespace/segment";
  static constexpr const char* kSegmentServiceType = "maliput_ros_interfaces/srv/Segment";
  static constexpr const char* kToRoadPositionServiceName = "/my_namespace/to_road_position";
  static constexpr const char* kToRoadPositionServiceType = "maliput_ros_interfaces/srv/ToRoadPosition";
  static constexpr const char* kFindRoadPositionsServiceName = "/my_namespace/find_road_positions";
  static constexpr const char* kFindRoadPositionsServiceType = "maliput_ros_interfaces/srv/FindRoadPositions";
  static constexpr const char* kToInertialPoseServiceName = "/my_namespace/to_inertial_pose";
  static constexpr const char* kToInertialPoseServiceType = "maliput_ros_interfaces/srv/ToInertialPose";
  static constexpr const char* kEvalMotionDerivativesServiceName = "/my_namespace/eval_motion_derivatives";
  static constexpr const char* kEvalMotionDerivativesServiceType = "maliput_ros_interfaces/srv/EvalMotionDerivatives";
  static constexpr const char* kDeriveLaneSRoutesServiceName = "/my_namespace/derive_lane_s_routes";
  static constexpr const char* kDeriveLaneSRoutesServiceType = "maliput_ros_interfaces/srv/DeriveLaneSRoutes";
  static constexpr const char* kSampleLaneSRouteServiceName = "/my_namespace/sample_lane_s_route";
  static constexpr const char* kSampleLaneSRouteServiceType = "maliput_ros_interfaces/srv/SampleLaneSRoute";

  const std::string kYamlFilePath{TEST_YAML_CONFIGURATION_PLUGIN_INSTALL_PATH};
  const std::chrono::nanoseconds kTimeout = std::chrono::seconds(1);
  const std::chrono::nanoseconds kTimeoutServiceCall = std::chrono::seconds(1);
  const std::chrono::nanoseconds kSleepPeriod = std::chrono::milliseconds(100);

  void SetUp() override {
    MaliputQueryNodeTest::SetUp();

    // Configure the mock RoadNetwork plugin.
    auto road_network = MakeRoadNetworkMock();
    RegisterRoadNetworkForPlugin(std::move(road_network));
    road_network_ptrs_ = GetRoadNetworkMockPointers();

    // Configure the maliput plugin environment variable.
    back_up_env_ = maliput::common::Filesystem::get_env_path(kEnvName);
    ASSERT_TRUE(setenv(kEnvName, kRoadNetowrkMockPluginPath, kReplace) == 0);

    // Load the YAML configuration file for the node.
    dut_ = std::make_shared<MaliputQueryNode>(kNodeName, kNodeNamespace, kNodeOptions);
  }

  void TearDown() override {
    // Restore the maliput plugin environment variable.
    ASSERT_TRUE(setenv(kEnvName, back_up_env_.c_str(), kReplace) == 0);
    MaliputQueryNodeTest::TearDown();
  }

  // Transitions from UNCONFIGURED to INACTIVE by CONFIGURING the node.
  void TransitionToConfigureFromUnconfigured() {
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, dut_->get_current_state().id());
    auto ret = kCallbackFailure;
    dut_->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE), ret);
    ASSERT_EQ(kCallbackSuccess, ret);
  }

  // Transitions from INACTIVE to ACTIVE the node.
  void TransitionToActiveFromConfigured() {
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, dut_->get_current_state().id());
    auto ret = kCallbackFailure;
    dut_->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE), ret);
    ASSERT_EQ(kCallbackSuccess, ret);
  }

  template <typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> MakeAsyncRequestAndWait(
      const std::string& service_name, const std::chrono::nanoseconds& timeout_service_call,
      std::shared_ptr<typename ServiceT::Request> request) {
    auto service = dut_->create_client<ServiceT>(service_name);
    auto future_result = service->async_send_request(request);
    return future_result.wait_for(timeout_service_call) == std::future_status::ready
               ? future_result.get()
               : std::shared_ptr<typename ServiceT::Response>{};
  }

  RoadNetworkMockPointers road_network_ptrs_{};
  std::shared_ptr<MaliputQueryNode> dut_{};

 private:
  const rclcpp::NodeOptions kNodeOptions = rclcpp::NodeOptions().append_parameter_override(
      kYamlConfigurationPathParameterName, rclcpp::ParameterValue(kYamlFilePath));
  std::string back_up_env_{};
};

// This function waits for an event to happen in the node graph.
// It has been adapted from:
// https://github.com/ros2/rclcpp/blob/rolling/rclcpp_lifecycle/test/test_lifecycle_node.cpp#L40-L60
// @return true When @p predicate becomes true before the timeout. Otherwise, false.
inline bool WaitForEvent(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::function<bool()> predicate,
                         std::chrono::nanoseconds timeout, std::chrono::nanoseconds sleep_period) {
  auto start = std::chrono::steady_clock::now();
  std::chrono::microseconds time_slept(0);

  bool predicate_result{};
  while (!(predicate_result = predicate()) &&
         time_slept < std::chrono::duration_cast<std::chrono::microseconds>(timeout)) {
    rclcpp::Event::SharedPtr graph_event = node->get_graph_event();
    node->wait_for_graph_change(graph_event, sleep_period);
    time_slept = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start);
  }
  return predicate_result;
}

// This function waits for a service to become available in the node graph.
// It has been adapted from:
// https://github.com/ros2/rclcpp/blob/rolling/rclcpp_lifecycle/test/test_lifecycle_node.cpp#L62-L78
// @return true When @p service_name service becomes available before @p timeout by iteratively asking every @p
// sleep_period. Otherwise, false.
inline bool WaitForService(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string& service_name,
                           std::chrono::nanoseconds timeout, std::chrono::nanoseconds sleep_period) {
  return WaitForEvent(
      node,
      [node, service_name]() {
        const auto service_names_and_types = node->get_service_names_and_types();
        return service_names_and_types.end() != service_names_and_types.find(service_name);
      },
      timeout, sleep_period);
}

}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
