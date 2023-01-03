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
#include "maliput_ros/ros/maliput_query_node.h"

#include <functional>
#include <stdexcept>

#include <maliput/api/road_network.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/plugin/create_road_network.h>
#include <maliput_ros_translation/convert.h>

#include "maliput_ros/utils/yaml_parser.h"

namespace maliput_ros {
namespace ros {

MaliputQueryNode::MaliputQueryNode(const std::string& node_name, const std::string& namespace_,
                                   const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode(node_name, namespace_, options), is_active_(false) {
  RCLCPP_INFO(get_logger(), "MaliputQueryNode");
  // Initialize the parameter for the YAML file path configuration.
  rcl_interfaces::msg::ParameterDescriptor maliput_plugin_yaml_file_path_descriptor;
  maliput_plugin_yaml_file_path_descriptor.name = kYamlConfigurationPath;
  maliput_plugin_yaml_file_path_descriptor.description = kYamlConfigurationPathDescription;
  // TODO(agalbachicar): this has been enabled for testing. We should pass in the proper context
  //                     to the node to make it read only.
  maliput_plugin_yaml_file_path_descriptor.read_only = false;
  this->declare_parameter(maliput_plugin_yaml_file_path_descriptor.name, rclcpp::ParameterValue(std::string{}),
                          maliput_plugin_yaml_file_path_descriptor);
}

void MaliputQueryNode::RoadGeometryCallback(
    const std::shared_ptr<maliput_ros_interfaces::srv::RoadGeometry::Request>,
    std::shared_ptr<maliput_ros_interfaces::srv::RoadGeometry::Response> response) const {
  RCLCPP_INFO(get_logger(), "RoadGeometryCallback");
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "The node is not active yet.");
    return;
  }
  response->road_geometry = maliput_ros_translation::ToRosMessage(maliput_query_->road_geometry());
}

std::string MaliputQueryNode::GetMaliputYamlFilePath() const {
  return this->get_parameter(kYamlConfigurationPath).get_parameter_value().get<std::string>();
}

bool MaliputQueryNode::LoadMaliputQuery() {
  RCLCPP_INFO(get_logger(), "LoadMaliputQuery");
  RCLCPP_INFO(get_logger(), "File path: " + GetMaliputYamlFilePath());
  try {
    const maliput_ros::utils::MaliputRoadNetworkConfiguration configurations =
        maliput_ros::utils::LoadYamlConfigFile(GetMaliputYamlFilePath());
    std::unique_ptr<maliput::api::RoadNetwork> road_network =
        maliput::plugin::CreateRoadNetwork(configurations.backend_name, configurations.backend_parameters);
    maliput_query_ = std::make_unique<MaliputQuery>(std::move(road_network));
  } catch (std::runtime_error& e) {
    RCLCPP_ERROR(get_logger(), std::string{"Error loading maliput RoadNetwork: "} + e.what());
    return false;
  }
  return true;
}

void MaliputQueryNode::TearDownMaliputQuery() {
  RCLCPP_INFO(get_logger(), "TearDownMaliputQuery");
  maliput_query_.reset();
}

bool MaliputQueryNode::InitializeAllServices() {
  RCLCPP_INFO(get_logger(), "InitializeAllServices");
  road_geometry_srv_ = this->create_service<maliput_ros_interfaces::srv::RoadGeometry>(
      kRoadGeometryServiceName,
      std::bind(&MaliputQueryNode::RoadGeometryCallback, this, std::placeholders::_1, std::placeholders::_2));
  return true;
}

void MaliputQueryNode::TearDownAllServices() {
  RCLCPP_INFO(get_logger(), "TearDownAllServices");
  road_geometry_srv_.reset();
}

MaliputQueryNode::LifecyleNodeCallbackReturn MaliputQueryNode::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "on_activate");
  is_active_.store(true);
  return LifecyleNodeCallbackReturn::SUCCESS;
}

MaliputQueryNode::LifecyleNodeCallbackReturn MaliputQueryNode::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "on_deactivate");
  is_active_.store(false);
  return LifecyleNodeCallbackReturn::SUCCESS;
}

MaliputQueryNode::LifecyleNodeCallbackReturn MaliputQueryNode::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "on_configure");
  const bool configure_result = LoadMaliputQuery() && InitializeAllServices();
  return configure_result ? LifecyleNodeCallbackReturn::SUCCESS : LifecyleNodeCallbackReturn::FAILURE;
}

MaliputQueryNode::LifecyleNodeCallbackReturn MaliputQueryNode::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "on_cleanup");
  TearDownAllServices();
  TearDownMaliputQuery();
  return LifecyleNodeCallbackReturn::SUCCESS;
}

}  // namespace ros
}  // namespace maliput_ros
