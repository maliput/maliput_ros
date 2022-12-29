// BSD 3-Clause License
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

maliput_ros_interfaces::msg::BranchPointId ToRosMessage(const maliput::api::BranchPointId& branch_point_id);
maliput_ros_interfaces::msg::JunctionId ToRosMessage(const maliput::api::JunctionId& junction_id);
maliput_ros_interfaces::msg::LaneId ToRosMessage(const maliput::api::LaneId& lane_id);
maliput_ros_interfaces::msg::RoadGeometryId ToRosMessage(const maliput::api::RoadGeometryId& road_geometry_id);
maliput_ros_interfaces::msg::SegmentId ToRosMessage(const maliput::api::SegmentId& segment_id);

maliput::api::BranchPointId FromRosMessage(const maliput_ros_interfaces::msg::BranchPointId& branch_point_id);
maliput::api::JunctionId FromRosMessage(const maliput_ros_interfaces::msg::JunctionId& junction_id);
maliput::api::LaneId FromRosMessage(const maliput_ros_interfaces::msg::LaneId& lane_id);
maliput::api::RoadGeometryId FromRosMessage(const maliput_ros_interfaces::msg::RoadGeometryId& road_geometry_id);
maliput::api::SegmentId FromRosMessage(const maliput_ros_interfaces::msg::SegmentId& segment_id);

maliput_ros_interfaces::msg::BranchPoint ToRosMessage(const maliput::api::BranchPoint* branch_point);
maliput_ros_interfaces::msg::Lane ToRosMessage(const maliput::api::Lane* lane);
maliput_ros_interfaces::msg::LaneEnd ToRosMessage(const maliput::api::LaneEnd& lane_end);
maliput_ros_interfaces::msg::LaneEndSet ToRosMessage(const maliput::api::LaneEndSet* lane_end_set);
maliput_ros_interfaces::msg::Junction ToRosMessage(const maliput::api::Junction* junction);
maliput_ros_interfaces::msg::RoadGeometry ToRosMessage(const maliput::api::RoadGeometry* road_geometry);
maliput_ros_interfaces::msg::Segment ToRosMessage(const maliput::api::Segment* segment);

}  // namespace maliput_ros_translation
