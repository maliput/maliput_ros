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
#include <maliput_ros_interfaces/srv/junction.hpp>
#include <maliput_ros_interfaces/srv/road_geometry.hpp>
#include <rcl_lifecycle/rcl_lifecycle.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "maliput_ros/ros/maliput_mock.h"
#include "maliput_ros/ros/maliput_plugin_config_test.h"
#include "maliput_ros_translation/convert.h"

namespace maliput_ros {
namespace ros {
namespace test {
namespace {

using ::testing::Return;
using ::testing::ReturnRef;

// @return A maliput::api::RoadNetwork populated with gmock versions of it, except maliput::api::rules::RuleRegistry.
std::unique_ptr<maliput::api::RoadNetwork> MakeRoadNetworkMock() {
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
  static constexpr const char* kJunctionServiceName = "/my_namespace/junction";
  static constexpr const char* kJunctionServiceType = "maliput_ros_interfaces/srv/Junction";

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
bool WaitForEvent(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, std::function<bool()> predicate,
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
bool WaitForService(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string& service_name,
                    std::chrono::nanoseconds timeout, std::chrono::nanoseconds sleep_period) {
  return WaitForEvent(
      node,
      [node, service_name]() {
        const auto service_names_and_types = node->get_service_names_and_types();
        return service_names_and_types.end() != service_names_and_types.find(service_name);
      },
      timeout, sleep_period);
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
  ASSERT_STREQ(service_names_and_types[kJunctionServiceName][0].c_str(), kJunctionServiceType);
}

// Makes sure services don't process the request when the node is not ACTIVE.
TEST_F(MaliputQueryNodeAfterConfigurationTest, CallingServiceBeforeActiveYieldsToFailure) {
  AddNodeToExecutorAndSpin(dut_);
  TransitionToConfigureFromUnconfigured();

  {
    ASSERT_TRUE(WaitForService(dut_, kRoadGeometryServiceName, kTimeout, kSleepPeriod));

    auto road_geometry_service =
        dut_->create_client<maliput_ros_interfaces::srv::RoadGeometry>(kRoadGeometryServiceName);
    auto request = std::make_shared<maliput_ros_interfaces::srv::RoadGeometry::Request>();
    auto future_result = road_geometry_service->async_send_request(request);
    const auto future_status = future_result.wait_for(kTimeoutServiceCall);

    ASSERT_TRUE(future_status == std::future_status::ready);
    const auto response = future_result.get();
    ASSERT_TRUE(response->road_geometry.id.id.empty());
  }
  {
    ASSERT_TRUE(WaitForService(dut_, kJunctionServiceName, kTimeout, kSleepPeriod));

    auto service = dut_->create_client<maliput_ros_interfaces::srv::Junction>(kJunctionServiceName);
    auto request = std::make_shared<maliput_ros_interfaces::srv::Junction::Request>();
    auto future_result = service->async_send_request(request);
    const auto future_status = future_result.wait_for(kTimeoutServiceCall);

    ASSERT_TRUE(future_status == std::future_status::ready);
    const auto response = future_result.get();
    ASSERT_TRUE(response->junction.id.id.empty());
  }
}

// Makes sure the node can transtion to the ACTIVE state.
TEST_F(MaliputQueryNodeAfterConfigurationTest, TransitionToActiveIsSuccessful) {
  TransitionToConfigureFromUnconfigured();
  TransitionToActiveFromConfigured();
}

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

  auto road_geometry_service = dut_->create_client<maliput_ros_interfaces::srv::RoadGeometry>(kRoadGeometryServiceName);
  ASSERT_TRUE(road_geometry_service->wait_for_service(kTimeout));
  auto request = std::make_shared<maliput_ros_interfaces::srv::RoadGeometry::Request>();
  auto future_result = road_geometry_service->async_send_request(request);
  auto future_status = future_result.wait_for(kTimeoutServiceCall);
  ASSERT_TRUE(future_status == std::future_status::ready);
  const auto response = future_result.get();

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

  auto junction_service = dut_->create_client<maliput_ros_interfaces::srv::Junction>(kJunctionServiceName);
  ASSERT_TRUE(junction_service->wait_for_service(kTimeout));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Junction::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kJunctionId);
  auto future_result = junction_service->async_send_request(request);
  auto future_status = future_result.wait_for(kTimeoutServiceCall);
  ASSERT_TRUE(future_status == std::future_status::ready);
  const auto response = future_result.get();

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

  auto junction_service = dut_->create_client<maliput_ros_interfaces::srv::Junction>(kJunctionServiceName);
  ASSERT_TRUE(junction_service->wait_for_service(kTimeout));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Junction::Request>();
  request->id = maliput_ros_translation::ToRosMessage(kJunctionId);
  auto future_result = junction_service->async_send_request(request);
  auto future_status = future_result.wait_for(kTimeoutServiceCall);
  ASSERT_TRUE(future_status == std::future_status::ready);
  const auto response = future_result.get();

  ASSERT_TRUE(response->junction.id.id.empty());
}

TEST_F(JunctionByIdServiceCallTest, EmptyIdReturnsEmptyResponse) {
  auto junction_service = dut_->create_client<maliput_ros_interfaces::srv::Junction>(kJunctionServiceName);
  ASSERT_TRUE(junction_service->wait_for_service(kTimeout));
  auto request = std::make_shared<maliput_ros_interfaces::srv::Junction::Request>();
  request->id.id = "";
  auto future_result = junction_service->async_send_request(request);
  auto future_status = future_result.wait_for(kTimeoutServiceCall);
  ASSERT_TRUE(future_status == std::future_status::ready);
  const auto response = future_result.get();

  ASSERT_TRUE(response->junction.id.id.empty());
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
