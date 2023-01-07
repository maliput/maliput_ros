// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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

#include <atomic>
#include <memory>
#include <string>
#include <utility>

#include <maliput_ros_interfaces/srv/junction.hpp>
#include <maliput_ros_interfaces/srv/road_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "maliput_ros/ros/maliput_query.h"

namespace maliput_ros {
namespace ros {

/// MaliputQueryNode is a LifecycleNode that serves as proxy to a maliput::api::RoadNetwork
/// set of queries.
///
/// It defines the following ROS parameters:
/// - yaml_configuration_path: a string which contains the path to a YAML file. The YAML is parsed
///   with maliput_ros::utils::LoadYamlConfigFile() to obtain a maliput_ros::utils::MaliputRoadNetworkConfiguration
///   and then use it to create a maliput::api::RoadNetwork from it. The YAML file must have the following structure:
/// @code {.yaml}
/// maliput:"
///   backend: <backend_name> "
///   parameters: "
///      <key_1>: <value_1> "
///      <key_2>: <value_2> "
///      // ...
///      <key_N>: <value_N> "
/// @endcode
///
/// The following depicts what this node does on each state transtion:
/// - MaliputQueryNode::on_configure(): the maliput::api::RoadNetwork, the MaliputQuery and the services are created.
/// - MaliputQueryNode::on_activate(): the services are enabled to respond to queries.
/// - MaliputQueryNode::on_deactivate(): the services are disabled to respond to queries.
/// - MaliputQueryNode::on_cleanup(): the maliput::api::RoadNetwork, the MaliputQuery, and the services are torn down.
///
/// This query server offers:
/// - /road_geometry: responds the maliput::api::RoadGeometry configuration.
class MaliputQueryNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  using LifecyleNodeCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Create a node based on the node name, namespace and rclcpp::Context.
  ///
  /// @param[in] node_name Name of the node.
  /// @param[in] namespace_ Namespace of the node.
  /// @param[in] options Additional options to control creation of the node.
  MaliputQueryNode(const std::string& node_name, const std::string& namespace_ = "",
                   const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  static constexpr const char* kJunctionServiceName = "junction";
  static constexpr const char* kRoadGeometryServiceName = "road_geometry";
  static constexpr const char* kYamlConfigurationPath = "yaml_configuration_path";
  static constexpr const char* kYamlConfigurationPathDescription =
      "File path to the yaml file containing the maliput plugin RoadNework loader.";

  // @return The path to the YAMl file containing the maliput plugin configuration from the node parameter.
  std::string GetMaliputYamlFilePath() const;

  // @brief Responds the maliput::api::Junction configuration.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Holds the maliput::api::JunctionId to query.
  // @param[out] response Loads the maliput::api::Junction description.
  void JunctionCallback(const std::shared_ptr<maliput_ros_interfaces::srv::Junction::Request> request,
                        std::shared_ptr<maliput_ros_interfaces::srv::Junction::Response> response) const;

  // @brief Responds the maliput::api::RoadGeometry configuration.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Unused.
  // @param[out] response Loads the maliput::api::RoadGeometry description.
  void RoadGeometryCallback(const std::shared_ptr<maliput_ros_interfaces::srv::RoadGeometry::Request> request,
                            std::shared_ptr<maliput_ros_interfaces::srv::RoadGeometry::Response> response) const;

  // @brief Loads the maliput::api::RoadNetwork from the yaml_configuration_path contents.
  // @return true When the load procedure is successful.
  bool LoadMaliputQuery();

  // @brief Deletes the maliput::api::RoadNetwork and the MaliputQuery.
  void TearDownMaliputQuery();

  // @brief Creates all services of this node.
  // @return true
  bool InitializeAllServices();

  // @brief Creates all services of this node.
  // @return true
  void TearDownAllServices();

  // @brief Loads the maliput_query_ and the services.
  // @param[in] state Unused.
  // @return LifecyleNodeCallbackReturn::SUCCESS When the load procedure is successful.
  // Otherwise, LifecyleNodeCallbackReturn::FAILURE.
  LifecyleNodeCallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  // @brief Enables queries to all services.
  // @param[in] state Unused.
  // @return LifecyleNodeCallbackReturn::SUCCESS.
  LifecyleNodeCallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  // @brief Disables queries to all services.
  // @param[in] state Unused.
  // @return LifecyleNodeCallbackReturn::SUCCESS.
  LifecyleNodeCallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  // @brief Deletes all services and deletes maliput_query_.
  // @param[in] state Unused.
  // @return LifecyleNodeCallbackReturn::SUCCESS.
  LifecyleNodeCallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  // @brief No-op.
  // @param[in] state Unused.
  // @return LifecyleNodeCallbackReturn::SUCCESS.
  LifecyleNodeCallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override {
    RCLCPP_INFO(get_logger(), "on_shutdown");
    return LifecyleNodeCallbackReturn::SUCCESS;
  }

  // Works as a barrier to all service callbacks. When it is true, callbacks can operate.
  std::atomic<bool> is_active_;
  // /junction service.
  rclcpp::Service<maliput_ros_interfaces::srv::Junction>::SharedPtr junction_srv_;
  // /road_geometry service.
  rclcpp::Service<maliput_ros_interfaces::srv::RoadGeometry>::SharedPtr road_geometry_srv_;
  // Proxy to a maliput::api::RoadNetwork queries.
  std::unique_ptr<maliput_ros::ros::MaliputQuery> maliput_query_;
};

}  // namespace ros
}  // namespace maliput_ros
