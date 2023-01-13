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

#include <algorithm>
#include <functional>
#include <stdexcept>

#include <maliput/api/junction.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_network.h>
#include <maliput/api/segment.h>
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
  maliput_plugin_yaml_file_path_descriptor.read_only = true;
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

void MaliputQueryNode::BranchPointCallback(
    const std::shared_ptr<maliput_ros_interfaces::srv::BranchPoint::Request> request,
    std::shared_ptr<maliput_ros_interfaces::srv::BranchPoint::Response> response) const {
  RCLCPP_INFO(get_logger(), "BranchPointCallback");
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "The node is not active yet.");
    return;
  }
  if (request->id.id.empty()) {
    RCLCPP_ERROR(get_logger(), "Request /branch_point with invalid value for BranchPointId.");
    return;
  }
  response->branch_point = maliput_ros_translation::ToRosMessage(
      maliput_query_->GetBranchPointBy(maliput_ros_translation::FromRosMessage(request->id)));
}

void MaliputQueryNode::FindRoadPositionsCallback(
    const std::shared_ptr<maliput_ros_interfaces::srv::FindRoadPositions::Request> request,
    std::shared_ptr<maliput_ros_interfaces::srv::FindRoadPositions::Response> response) const {
  RCLCPP_INFO(get_logger(), "FindRoadPositionsCallback");
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "The node is not active yet.");
    return;
  }
  const std::vector<maliput::api::RoadPositionResult> road_position_results = maliput_query_->FindRoadPositions(
      maliput_ros_translation::FromRosMessage(request->inertial_position), request->radius);
  response->road_position_results.resize(road_position_results.size());
  std::transform(road_position_results.cbegin(), road_position_results.cend(), response->road_position_results.begin(),
                 [](const maliput::api::RoadPositionResult& road_position_result) {
                   return maliput_ros_translation::ToRosMessage(road_position_result);
                 });
}

void MaliputQueryNode::JunctionCallback(
    const std::shared_ptr<maliput_ros_interfaces::srv::Junction::Request> request,
    std::shared_ptr<maliput_ros_interfaces::srv::Junction::Response> response) const {
  RCLCPP_INFO(get_logger(), "JunctionCallback");
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "The node is not active yet.");
    return;
  }
  if (request->id.id.empty()) {
    RCLCPP_ERROR(get_logger(), "Request /junction with invalid value for JunctionId.");
    return;
  }
  response->junction = maliput_ros_translation::ToRosMessage(
      maliput_query_->GetJunctionBy(maliput_ros_translation::FromRosMessage(request->id)));
}

void MaliputQueryNode::LaneCallback(const std::shared_ptr<maliput_ros_interfaces::srv::Lane::Request> request,
                                    std::shared_ptr<maliput_ros_interfaces::srv::Lane::Response> response) const {
  RCLCPP_INFO(get_logger(), "LaneCallback");
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "The node is not active yet.");
    return;
  }
  if (request->id.id.empty()) {
    RCLCPP_ERROR(get_logger(), "Request /lane with invalid value for LaneId.");
    return;
  }
  response->lane = maliput_ros_translation::ToRosMessage(
      maliput_query_->GetLaneBy(maliput_ros_translation::FromRosMessage(request->id)));
}

void MaliputQueryNode::SegmentCallback(const std::shared_ptr<maliput_ros_interfaces::srv::Segment::Request> request,
                                       std::shared_ptr<maliput_ros_interfaces::srv::Segment::Response> response) const {
  RCLCPP_INFO(get_logger(), "SegmentCallback");
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "The node is not active yet.");
    return;
  }
  if (request->id.id.empty()) {
    RCLCPP_ERROR(get_logger(), "Request /segment with invalid value for SegmentId.");
    return;
  }
  response->segment = maliput_ros_translation::ToRosMessage(
      maliput_query_->GetSegmentBy(maliput_ros_translation::FromRosMessage(request->id)));
}

void MaliputQueryNode::ToRoadPositionCallback(
    const std::shared_ptr<maliput_ros_interfaces::srv::ToRoadPosition::Request> request,
    std::shared_ptr<maliput_ros_interfaces::srv::ToRoadPosition::Response> response) const {
  RCLCPP_INFO(get_logger(), "ToRoadPositionCallback");
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "The node is not active yet.");
    return;
  }
  response->road_position_result = maliput_ros_translation::ToRosMessage(
      maliput_query_->ToRoadPosition(maliput_ros_translation::FromRosMessage(request->inertial_position)));
}

void MaliputQueryNode::ToInertialPoseCallback(
    const std::shared_ptr<maliput_ros_interfaces::srv::ToInertialPose::Request> request,
    std::shared_ptr<maliput_ros_interfaces::srv::ToInertialPose::Response> response) const {
  RCLCPP_INFO(get_logger(), "ToInertialPoseCallback");
  if (!is_active_.load()) {
    RCLCPP_WARN(get_logger(), "The node is not active yet.");
    return;
  }
  const maliput::api::RoadGeometry* road_geometry = maliput_query_->road_geometry();
  const std::optional<std::pair<maliput::api::InertialPosition, maliput::api::Rotation>> optional_inertial_pose =
      maliput_query_->ToInertialPose(maliput_ros_translation::FromRosMessage(road_geometry, request->road_position));
  if (!optional_inertial_pose.has_value()) {
    RCLCPP_WARN(get_logger(), "Unknown RoadPosition.");
    return;
  }
  response->position = maliput_ros_translation::ToRosMessage(optional_inertial_pose->first);
  response->orientation = maliput_ros_translation::ToRosMessage(optional_inertial_pose->second);
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

  branch_point_srv_ = this->create_service<maliput_ros_interfaces::srv::BranchPoint>(
      kBranchPointServiceName,
      std::bind(&MaliputQueryNode::BranchPointCallback, this, std::placeholders::_1, std::placeholders::_2));
  find_road_positions_srv_ = this->create_service<maliput_ros_interfaces::srv::FindRoadPositions>(
      kFindRoadPositionsServiceName,
      std::bind(&MaliputQueryNode::FindRoadPositionsCallback, this, std::placeholders::_1, std::placeholders::_2));
  junction_srv_ = this->create_service<maliput_ros_interfaces::srv::Junction>(
      kJunctionServiceName,
      std::bind(&MaliputQueryNode::JunctionCallback, this, std::placeholders::_1, std::placeholders::_2));
  lane_srv_ = this->create_service<maliput_ros_interfaces::srv::Lane>(
      kLaneServiceName, std::bind(&MaliputQueryNode::LaneCallback, this, std::placeholders::_1, std::placeholders::_2));
  road_geometry_srv_ = this->create_service<maliput_ros_interfaces::srv::RoadGeometry>(
      kRoadGeometryServiceName,
      std::bind(&MaliputQueryNode::RoadGeometryCallback, this, std::placeholders::_1, std::placeholders::_2));
  segment_srv_ = this->create_service<maliput_ros_interfaces::srv::Segment>(
      kSegmentServiceName,
      std::bind(&MaliputQueryNode::SegmentCallback, this, std::placeholders::_1, std::placeholders::_2));
  to_road_position_srv_ = this->create_service<maliput_ros_interfaces::srv::ToRoadPosition>(
      kToRoadPositionServiceName,
      std::bind(&MaliputQueryNode::ToRoadPositionCallback, this, std::placeholders::_1, std::placeholders::_2));
  to_inertial_pose_srv_ = this->create_service<maliput_ros_interfaces::srv::ToInertialPose>(
      kToInertialPoseServiceName,
      std::bind(&MaliputQueryNode::ToInertialPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  return true;
}

void MaliputQueryNode::TearDownAllServices() {
  RCLCPP_INFO(get_logger(), "TearDownAllServices");
  branch_point_srv_.reset();
  find_road_positions_srv_.reset();
  junction_srv_.reset();
  lane_srv_.reset();
  road_geometry_srv_.reset();
  segment_srv_.reset();
  to_road_position_srv_.reset();
  to_inertial_pose_srv_.reset();
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
