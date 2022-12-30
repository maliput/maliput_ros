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

#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>
#include <maliput_ros_interfaces/msg/branch_point.hpp>
#include <maliput_ros_interfaces/msg/branch_point_id.hpp>
#include <maliput_ros_interfaces/msg/junction.hpp>
#include <maliput_ros_interfaces/msg/junction_id.hpp>
#include <maliput_ros_interfaces/msg/lane.hpp>
#include <maliput_ros_interfaces/msg/lane_end.hpp>
#include <maliput_ros_interfaces/msg/lane_end_set.hpp>
#include <maliput_ros_interfaces/msg/lane_id.hpp>
#include <maliput_ros_interfaces/msg/road_geometry.hpp>
#include <maliput_ros_interfaces/msg/road_geometry_id.hpp>
#include <maliput_ros_interfaces/msg/segment.hpp>
#include <maliput_ros_interfaces/msg/segment_id.hpp>

namespace maliput_ros_translation {

/// Converts a maliput::api::BranchPointId into maliput_ros_interfaces::msg::BranchPointId.
/// @param branch_point_id The ID to convert.
/// @return A maliput_ros_interfaces::msg::BranchPointId.
maliput_ros_interfaces::msg::BranchPointId ToRosMessage(const maliput::api::BranchPointId& branch_point_id);

/// Converts a maliput::api::JunctionId into maliput_ros_interfaces::msg::JunctionId.
/// @param junction_id The ID to convert.
/// @return A maliput_ros_interfaces::msg::JunctionId.
maliput_ros_interfaces::msg::JunctionId ToRosMessage(const maliput::api::JunctionId& junction_id);

/// Converts a maliput::api::LaneId into maliput_ros_interfaces::msg::LaneId.
/// @param lane_id The ID to convert.
/// @return A maliput_ros_interfaces::msg::LaneId.
maliput_ros_interfaces::msg::LaneId ToRosMessage(const maliput::api::LaneId& lane_id);

/// Converts a maliput::api::RoadGeometryId into maliput_ros_interfaces::msg::RoadGeometryId.
/// @param road_geometry_id The ID to convert.
/// @return A maliput_ros_interfaces::msg::RoadGeometryId.
maliput_ros_interfaces::msg::RoadGeometryId ToRosMessage(const maliput::api::RoadGeometryId& road_geometry_id);

/// Converts a maliput::api::SegmentId into maliput_ros_interfaces::msg::SegmentId.
/// @param segment_id The ID to convert.
/// @return A maliput_ros_interfaces::msg::SegmentId.
maliput_ros_interfaces::msg::SegmentId ToRosMessage(const maliput::api::SegmentId& segment_id);

/// Converts a maliput_ros_interfaces::msg::BranchPointId into maliput::api::BranchPointId.
/// @param branch_point_id The ID to convert.
/// @return A maliput::api::BranchPointId.
maliput::api::BranchPointId FromRosMessage(const maliput_ros_interfaces::msg::BranchPointId& branch_point_id);

/// Converts a maliput_ros_interfaces::msg::JunctionId into maliput::api::JunctionId.
/// @param junction_id The ID to convert.
/// @return A maliput::api::JunctionId.
maliput::api::JunctionId FromRosMessage(const maliput_ros_interfaces::msg::JunctionId& junction_id);

/// Converts a maliput_ros_interfaces::msg::LaneId into maliput::api::LaneId.
/// @param lane_id The ID to convert.
/// @return A maliput::api::LaneId.
maliput::api::LaneId FromRosMessage(const maliput_ros_interfaces::msg::LaneId& lane_id);

/// Converts a maliput_ros_interfaces::msg::RoadGeometryId into maliput::api::RoadGeometryId.
/// @param road_geometry_id The ID to convert.
/// @return A maliput::api::RoadGeometryId.
maliput::api::RoadGeometryId FromRosMessage(const maliput_ros_interfaces::msg::RoadGeometryId& road_geometry_id);

/// Converts a maliput_ros_interfaces::msg::SegmentId into maliput::api::SegmentId.
/// @param segment_id The ID to convert.
/// @return A maliput::api::SegmentId.
maliput::api::SegmentId FromRosMessage(const maliput_ros_interfaces::msg::SegmentId& segment_id);

/// Converts a maliput::api::BranchPoint into maliput_ros_interfaces::msg::BranchPoint.
/// When @p branch_point is nullptr, the returned message is uninitialized. Consumers may opt to
/// validate the ID which is is an empty string when @p branch_point is nullptr.
/// @param branch_point The entity to convert to a ROS message.
/// @return A maliput_ros_interfaces::msg::BranchPoint.
maliput_ros_interfaces::msg::BranchPoint ToRosMessage(const maliput::api::BranchPoint* branch_point);

/// Converts a maliput::api::Lane into maliput_ros_interfaces::msg::Lane.
/// When @p lane is nullptr, the returned message is uninitialized. Consumers may opt to
/// validate the ID which is is an empty string when @p lane is nullptr.
/// @param lane The entity to convert to a ROS message.
/// @return A maliput_ros_interfaces::msg::Lane.
maliput_ros_interfaces::msg::Lane ToRosMessage(const maliput::api::Lane* lane);

/// Converts a maliput::api::LaneEnd into maliput_ros_interfaces::msg::LaneEnd.
/// When @p lane_end.lane is nullptr, the returned message is uninitialized. Consumers may opt to
/// validate the Lane ID which is is an empty string when @p lane_end.lane is nullptr.
/// @param lane_end The entity to convert to a ROS message.
/// @return A maliput_ros_interfaces::msg::LaneEnd.
maliput_ros_interfaces::msg::LaneEnd ToRosMessage(const maliput::api::LaneEnd& lane_end);

/// Converts a maliput::api::LaneEndSet into maliput_ros_interfaces::msg::LaneEndSet.
/// When @p lane_end_set is nullptr, the returned message is uninitialized. Unfortunately,
/// there is aliasing between an empty LaneEndSet and the value produced by this function
/// when it receives a nullptr.
/// @param lane_end_set The entity to convert to a ROS message.
/// @return A maliput_ros_interfaces::msg::LaneEndSet.
maliput_ros_interfaces::msg::LaneEndSet ToRosMessage(const maliput::api::LaneEndSet* lane_end_set);

/// Converts a maliput::api::Junction into maliput_ros_interfaces::msg::Junction.
/// When @p junction is nullptr, the returned message is uninitialized. Consumers may opt to
/// validate the ID which is is an empty string when @p junction is nullptr.
/// @param junction The entity to convert to a ROS message.
/// @return A maliput_ros_interfaces::msg::Junction.
maliput_ros_interfaces::msg::Junction ToRosMessage(const maliput::api::Junction* junction);

/// Converts a maliput::api::RoadGeometry into maliput_ros_interfaces::msg::RoadGeometry.
/// When @p road_geometry is nullptr, the returned message is uninitialized. Consumers may opt to
/// validate the ID which is is an empty string when @p road_geometry is nullptr.
/// @param road_geometry The entity to convert to a ROS message.
/// @return A maliput_ros_interfaces::msg::RoadGeometry.
maliput_ros_interfaces::msg::RoadGeometry ToRosMessage(const maliput::api::RoadGeometry* road_geometry);

/// Converts a maliput::api::Segment into maliput_ros_interfaces::msg::Segment.
/// When @p segment is nullptr, the returned message is uninitialized. Consumers may opt to
/// validate the ID which is is an empty string when @p segment is nullptr.
/// @param segment The entity to convert to a ROS message.
/// @return A maliput_ros_interfaces::msg::Segment.
maliput_ros_interfaces::msg::Segment ToRosMessage(const maliput::api::Segment* segment);

}  // namespace maliput_ros_translation
