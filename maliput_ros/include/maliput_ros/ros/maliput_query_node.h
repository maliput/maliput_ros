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

#include <maliput_ros_interfaces/srv/branch_point.hpp>
#include <maliput_ros_interfaces/srv/find_road_positions.hpp>
#include <maliput_ros_interfaces/srv/junction.hpp>
#include <maliput_ros_interfaces/srv/lane.hpp>
#include <maliput_ros_interfaces/srv/road_geometry.hpp>
#include <maliput_ros_interfaces/srv/segment.hpp>
#include <maliput_ros_interfaces/srv/to_inertial_pose.hpp>
#include <maliput_ros_interfaces/srv/to_road_position.hpp>
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
/// - /branch_point: looks for a maliput::api::BranchPoint by its ID.
/// - /find_road_positions: finds all maliput::api::RoadPositionResult in radius distance from a
/// maliput::api::InertialPosition.
/// - /junction: looks for a maliput::api::Junction by its ID.
/// - /lane: looks for a maliput::api::Lane by its ID.
/// - /road_geometry: responds the maliput::api::RoadGeometry configuration.
/// - /segment: looks for a maliput::api::Segment by its ID.
/// - /to_road_position: maps a maliput::api::InertialPosition into a maliput::api::RoadPosition.
/// - /to_inertial_pose: maps a maliput::api::RoadPosition into a maliput::api::InertialPosition and the
/// maliput::api::Rotation there.
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
  static constexpr const char* kBranchPointServiceName = "branch_point";
  static constexpr const char* kFindRoadPositionsServiceName = "find_road_positions";
  static constexpr const char* kJunctionServiceName = "junction";
  static constexpr const char* kLaneServiceName = "lane";
  static constexpr const char* kRoadGeometryServiceName = "road_geometry";
  static constexpr const char* kSegmentServiceName = "segment";
  static constexpr const char* kToInertialPoseServiceName = "to_inertial_pose";
  static constexpr const char* kToRoadPositionServiceName = "to_road_position";
  static constexpr const char* kYamlConfigurationPath = "yaml_configuration_path";
  static constexpr const char* kYamlConfigurationPathDescription =
      "File path to the yaml file containing the maliput plugin RoadNework loader.";

  // @return The path to the YAMl file containing the maliput plugin configuration from the node parameter.
  std::string GetMaliputYamlFilePath() const;

  // @brief Responds the maliput::api::BranchPoint configuration.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Holds the maliput::api::BranchPointId to query.
  // @param[out] response Loads the maliput::api::BranchPoint description.
  void BranchPointCallback(const std::shared_ptr<maliput_ros_interfaces::srv::BranchPoint::Request> request,
                           std::shared_ptr<maliput_ros_interfaces::srv::BranchPoint::Response> response) const;

  // @brief Responds all the maliput::api::RoadPositionResults within a given radius distance from a
  // maliput::api::InertialPosition.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Holds the maliput::api::InertialPosition and radius to find all the maliput::api::RoadPosition.
  // @param[out] response Holds all the maliput::api::RoadPositionResults of the transformation.
  void FindRoadPositionsCallback(
      const std::shared_ptr<maliput_ros_interfaces::srv::FindRoadPositions::Request> request,
      std::shared_ptr<maliput_ros_interfaces::srv::FindRoadPositions::Response> response) const;

  // @brief Responds the maliput::api::Junction configuration.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Holds the maliput::api::JunctionId to query.
  // @param[out] response Loads the maliput::api::Junction description.
  void JunctionCallback(const std::shared_ptr<maliput_ros_interfaces::srv::Junction::Request> request,
                        std::shared_ptr<maliput_ros_interfaces::srv::Junction::Response> response) const;

  // @brief Responds the maliput::api::Lane configuration.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Holds the maliput::api::LaneId to query.
  // @param[out] response Loads the maliput::api::Lane description.
  void LaneCallback(const std::shared_ptr<maliput_ros_interfaces::srv::Lane::Request> request,
                    std::shared_ptr<maliput_ros_interfaces::srv::Lane::Response> response) const;

  // @brief Responds the maliput::api::RoadGeometry configuration.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Unused.
  // @param[out] response Loads the maliput::api::RoadGeometry description.
  void RoadGeometryCallback(const std::shared_ptr<maliput_ros_interfaces::srv::RoadGeometry::Request> request,
                            std::shared_ptr<maliput_ros_interfaces::srv::RoadGeometry::Response> response) const;

  // @brief Responds the maliput::api::Segment configuration.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Holds the maliput::api::SegmentId to query.
  // @param[out] response Loads the maliput::api::Segment description.
  void SegmentCallback(const std::shared_ptr<maliput_ros_interfaces::srv::Segment::Request> request,
                       std::shared_ptr<maliput_ros_interfaces::srv::Segment::Response> response) const;

  // @brief Responds the maliput::api::RoadPositionResult transformation given a maliput::api::InertialPosition.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Holds the maliput::api::InertialPosition to transform into a maliput::api::RoadPosition.
  // @param[out] response Holds the maliput::api::RoadPositionResult of the transformation.
  void ToRoadPositionCallback(const std::shared_ptr<maliput_ros_interfaces::srv::ToRoadPosition::Request> request,
                              std::shared_ptr<maliput_ros_interfaces::srv::ToRoadPosition::Response> response) const;

  // @brief Computes the INERTIAL Frame transformation of a maliput::api::RoadPosition.
  // @pre The node must be in the ACTIVE state.
  // @param[in] request Holds the maliput::api::RoadPosition to transform into a maliput::api::InertialPosition and a
  // maliput::api::Rotation.
  // @param[out] response Holds the maliput::api::InertialPosition and the maliput::api::Rotation.
  void ToInertialPoseCallback(const std::shared_ptr<maliput_ros_interfaces::srv::ToInertialPose::Request> request,
                              std::shared_ptr<maliput_ros_interfaces::srv::ToInertialPose::Response> response) const;

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
  // /branch_point service.
  rclcpp::Service<maliput_ros_interfaces::srv::BranchPoint>::SharedPtr branch_point_srv_;
  // /find_road_positions service.
  rclcpp::Service<maliput_ros_interfaces::srv::FindRoadPositions>::SharedPtr find_road_positions_srv_;
  // /junction service.
  rclcpp::Service<maliput_ros_interfaces::srv::Junction>::SharedPtr junction_srv_;
  // /lane service.
  rclcpp::Service<maliput_ros_interfaces::srv::Lane>::SharedPtr lane_srv_;
  // /road_geometry service.
  rclcpp::Service<maliput_ros_interfaces::srv::RoadGeometry>::SharedPtr road_geometry_srv_;
  // /segment service.
  rclcpp::Service<maliput_ros_interfaces::srv::Segment>::SharedPtr segment_srv_;
  // /to_road_position service.
  rclcpp::Service<maliput_ros_interfaces::srv::ToRoadPosition>::SharedPtr to_road_position_srv_;
  // /to_inertial_pose service.
  rclcpp::Service<maliput_ros_interfaces::srv::ToInertialPose>::SharedPtr to_inertial_pose_srv_;
  // Proxy to a maliput::api::RoadNetwork queries.
  std::unique_ptr<maliput_ros::ros::MaliputQuery> maliput_query_;
};

}  // namespace ros
}  // namespace maliput_ros
