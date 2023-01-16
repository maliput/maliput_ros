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
#include "maliput_ros_translation/convert.h"

#include <algorithm>
#include <optional>

#include <maliput/common/maliput_throw.h>
#include <maliput/math/quaternion.h>
#include <maliput/math/vector.h>

namespace maliput_ros_translation {
namespace {

template <typename To, typename From>
inline To ConvertTypeSpecificIdentifierToRosMessage(const From& from) {
  To to;
  to.id = from.string();
  return to;
}

template <typename To, typename From>
inline To ConvertRosMessageToTypeSpecificIdentifier(const From& from) {
  return To(from.id);
}

}  // namespace

maliput_ros_interfaces::msg::BranchPointId ToRosMessage(const maliput::api::BranchPointId& branch_point_id) {
  return ConvertTypeSpecificIdentifierToRosMessage<maliput_ros_interfaces::msg::BranchPointId>(branch_point_id);
}

maliput_ros_interfaces::msg::JunctionId ToRosMessage(const maliput::api::JunctionId& junction_id) {
  return ConvertTypeSpecificIdentifierToRosMessage<maliput_ros_interfaces::msg::JunctionId>(junction_id);
}

maliput_ros_interfaces::msg::LaneId ToRosMessage(const maliput::api::LaneId& lane_id) {
  return ConvertTypeSpecificIdentifierToRosMessage<maliput_ros_interfaces::msg::LaneId>(lane_id);
}

maliput_ros_interfaces::msg::RoadGeometryId ToRosMessage(const maliput::api::RoadGeometryId& road_geometry_id) {
  return ConvertTypeSpecificIdentifierToRosMessage<maliput_ros_interfaces::msg::RoadGeometryId>(road_geometry_id);
}

maliput_ros_interfaces::msg::SegmentId ToRosMessage(const maliput::api::SegmentId& segment_id) {
  return ConvertTypeSpecificIdentifierToRosMessage<maliput_ros_interfaces::msg::SegmentId>(segment_id);
}

maliput::api::BranchPointId FromRosMessage(const maliput_ros_interfaces::msg::BranchPointId& branch_point_id) {
  return ConvertRosMessageToTypeSpecificIdentifier<maliput::api::BranchPointId>(branch_point_id);
}

maliput::api::JunctionId FromRosMessage(const maliput_ros_interfaces::msg::JunctionId& junction_id) {
  return ConvertRosMessageToTypeSpecificIdentifier<maliput::api::JunctionId>(junction_id);
}

maliput::api::LaneId FromRosMessage(const maliput_ros_interfaces::msg::LaneId& lane_id) {
  return ConvertRosMessageToTypeSpecificIdentifier<maliput::api::LaneId>(lane_id);
}

maliput::api::RoadGeometryId FromRosMessage(const maliput_ros_interfaces::msg::RoadGeometryId& road_geometry_id) {
  return ConvertRosMessageToTypeSpecificIdentifier<maliput::api::RoadGeometryId>(road_geometry_id);
}

maliput::api::SegmentId FromRosMessage(const maliput_ros_interfaces::msg::SegmentId& segment_id) {
  return ConvertRosMessageToTypeSpecificIdentifier<maliput::api::SegmentId>(segment_id);
}

maliput_ros_interfaces::msg::BranchPoint ToRosMessage(const maliput::api::BranchPoint* branch_point) {
  maliput_ros_interfaces::msg::BranchPoint msg;
  if (branch_point == nullptr) {
    return msg;
  }
  msg.id = ToRosMessage(branch_point->id());
  msg.road_geometry_id = ToRosMessage(branch_point->road_geometry()->id());
  msg.a_side = ToRosMessage(branch_point->GetASide());
  msg.b_side = ToRosMessage(branch_point->GetBSide());
  return msg;
}

maliput_ros_interfaces::msg::Lane ToRosMessage(const maliput::api::Lane* lane) {
  maliput_ros_interfaces::msg::Lane msg;
  if (lane == nullptr) {
    return msg;
  }
  msg.id = ToRosMessage(lane->id());
  msg.segment_id = ToRosMessage(lane->segment()->id());
  msg.index = lane->index();
  if (lane->to_left() != nullptr) {
    msg.left_lane = ToRosMessage(lane->to_left()->id());
  }
  if (lane->to_right() != nullptr) {
    msg.right_lane = ToRosMessage(lane->to_right()->id());
  }
  msg.length = lane->length();
  msg.start_branch_point_id = ToRosMessage(lane->GetBranchPoint(maliput::api::LaneEnd::Which::kStart)->id());
  msg.finish_branch_point_id = ToRosMessage(lane->GetBranchPoint(maliput::api::LaneEnd::Which::kFinish)->id());
  const std::optional<maliput::api::LaneEnd> default_start_branch =
      lane->GetDefaultBranch(maliput::api::LaneEnd::Which::kStart);
  if (default_start_branch.has_value()) {
    msg.default_start_branch = ToRosMessage(default_start_branch.value());
  }
  const std::optional<maliput::api::LaneEnd> default_finish_branch =
      lane->GetDefaultBranch(maliput::api::LaneEnd::Which::kFinish);
  if (default_finish_branch.has_value()) {
    msg.default_finish_branch = ToRosMessage(default_finish_branch.value());
  }
  return msg;
}

maliput_ros_interfaces::msg::LaneEnd ToRosMessage(const maliput::api::LaneEnd& lane_end) {
  maliput_ros_interfaces::msg::LaneEnd msg;
  if (lane_end.lane == nullptr) {
    return msg;
  }
  msg.lane_id = ToRosMessage(lane_end.lane->id());
  msg.end = lane_end.end == maliput::api::LaneEnd::Which::kStart ? maliput_ros_interfaces::msg::LaneEnd::WHICHSTART
                                                                 : maliput_ros_interfaces::msg::LaneEnd::WHICHFINISH;
  return msg;
}

maliput_ros_interfaces::msg::LaneEndSet ToRosMessage(const maliput::api::LaneEndSet* lane_end_set) {
  maliput_ros_interfaces::msg::LaneEndSet msg;
  if (lane_end_set == nullptr) {
    return msg;
  }
  for (int i = 0; i < lane_end_set->size(); ++i) {
    msg.lane_ends.push_back(ToRosMessage(lane_end_set->get(i)));
  }
  return msg;
}

maliput_ros_interfaces::msg::Junction ToRosMessage(const maliput::api::Junction* junction) {
  maliput_ros_interfaces::msg::Junction msg;
  if (junction == nullptr) {
    return msg;
  }
  msg.id = ToRosMessage(junction->id());
  msg.road_geometry_id = ToRosMessage(junction->road_geometry()->id());
  for (int i = 0; i < junction->num_segments(); ++i) {
    msg.segment_ids.push_back(ToRosMessage(junction->segment(i)->id()));
  }
  return msg;
}

maliput_ros_interfaces::msg::RoadGeometry ToRosMessage(const maliput::api::RoadGeometry* road_geometry) {
  maliput_ros_interfaces::msg::RoadGeometry msg;
  if (road_geometry == nullptr) {
    return msg;
  }
  msg.id = ToRosMessage(road_geometry->id());
  for (int i = 0; i < road_geometry->num_junctions(); ++i) {
    msg.junction_ids.push_back(ToRosMessage(road_geometry->junction(i)->id()));
  }
  for (int i = 0; i < road_geometry->num_branch_points(); ++i) {
    msg.branch_point_ids.push_back(ToRosMessage(road_geometry->branch_point(i)->id()));
  }
  msg.linear_tolerance = road_geometry->linear_tolerance();
  msg.angular_tolerance = road_geometry->angular_tolerance();
  msg.scale_length = road_geometry->scale_length();
  const maliput::math::Vector3 translation = road_geometry->inertial_to_backend_frame_translation();
  msg.inertial_to_backend_frame_translation.x = translation.x();
  msg.inertial_to_backend_frame_translation.y = translation.y();
  msg.inertial_to_backend_frame_translation.z = translation.z();
  return msg;
}

maliput_ros_interfaces::msg::Segment ToRosMessage(const maliput::api::Segment* segment) {
  maliput_ros_interfaces::msg::Segment msg;
  if (segment == nullptr) {
    return msg;
  }
  msg.id = ToRosMessage(segment->id());
  msg.junction_id = ToRosMessage(segment->junction()->id());
  for (int i = 0; i < segment->num_lanes(); ++i) {
    msg.lane_ids.push_back(ToRosMessage(segment->lane(i)->id()));
  }
  return msg;
}

maliput_ros_interfaces::msg::InertialPosition ToRosMessage(const maliput::api::InertialPosition& inertial_position) {
  maliput_ros_interfaces::msg::InertialPosition msg;
  msg.x = inertial_position.x();
  msg.y = inertial_position.y();
  msg.z = inertial_position.z();
  return msg;
}

maliput_ros_interfaces::msg::LanePosition ToRosMessage(const maliput::api::LanePosition& lane_position) {
  maliput_ros_interfaces::msg::LanePosition msg;
  msg.s = lane_position.s();
  msg.r = lane_position.r();
  msg.h = lane_position.h();
  return msg;
}

maliput_ros_interfaces::msg::RoadPosition ToRosMessage(const maliput::api::RoadPosition& road_position) {
  maliput_ros_interfaces::msg::RoadPosition msg;
  if (road_position.lane != nullptr) {
    msg.lane_id = ToRosMessage(road_position.lane->id());
    msg.pos = ToRosMessage(road_position.pos);
  }
  return msg;
}

maliput_ros_interfaces::msg::RoadPositionResult ToRosMessage(
    const maliput::api::RoadPositionResult& road_position_result) {
  maliput_ros_interfaces::msg::RoadPositionResult msg;
  msg.road_position = ToRosMessage(road_position_result.road_position);
  msg.nearest_position = ToRosMessage(road_position_result.nearest_position);
  msg.distance = road_position_result.distance;
  return msg;
}

maliput::api::InertialPosition FromRosMessage(const maliput_ros_interfaces::msg::InertialPosition& inertial_position) {
  return maliput::api::InertialPosition(inertial_position.x, inertial_position.y, inertial_position.z);
}

maliput::api::LanePosition FromRosMessage(const maliput_ros_interfaces::msg::LanePosition& lane_position) {
  return maliput::api::LanePosition(lane_position.s, lane_position.r, lane_position.h);
}

maliput::api::RoadPosition FromRosMessage(const maliput::api::RoadGeometry* road_geometry,
                                          const maliput_ros_interfaces::msg::RoadPosition& road_position) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  return road_position.lane_id.id.empty()
             ? maliput::api::RoadPosition()
             : maliput::api::RoadPosition(road_geometry->ById().GetLane(FromRosMessage(road_position.lane_id)),
                                          FromRosMessage(road_position.pos));
}

maliput_ros_interfaces::msg::Rotation ToRosMessage(const maliput::api::Rotation& rotation) {
  maliput_ros_interfaces::msg::Rotation msg;
  msg.x = rotation.quat().x();
  msg.y = rotation.quat().y();
  msg.z = rotation.quat().z();
  msg.w = rotation.quat().w();
  return msg;
}

maliput_ros_interfaces::msg::IsoLaneVelocity ToRosMessage(const maliput::api::IsoLaneVelocity& velocity) {
  maliput_ros_interfaces::msg::IsoLaneVelocity msg;
  msg.sigma_v = velocity.sigma_v;
  msg.rho_v = velocity.rho_v;
  msg.eta_v = velocity.eta_v;
  return msg;
}

maliput::api::IsoLaneVelocity FromRosMessage(const maliput_ros_interfaces::msg::IsoLaneVelocity& velocity) {
  return maliput::api::IsoLaneVelocity{velocity.sigma_v, velocity.rho_v, velocity.eta_v};
}

maliput_ros_interfaces::msg::HBounds ToRosMessage(const maliput::api::HBounds& h_bounds) {
  maliput_ros_interfaces::msg::HBounds msg;
  msg.min = h_bounds.min();
  msg.max = h_bounds.max();
  return msg;
}

maliput::api::HBounds FromRosMessage(const maliput_ros_interfaces::msg::HBounds& h_bounds) {
  return maliput::api::HBounds(h_bounds.min, h_bounds.max);
}

maliput_ros_interfaces::msg::RBounds ToRosMessage(const maliput::api::RBounds& r_bounds) {
  maliput_ros_interfaces::msg::RBounds msg;
  msg.min = r_bounds.min();
  msg.max = r_bounds.max();
  return msg;
}

maliput::api::RBounds FromRosMessage(const maliput_ros_interfaces::msg::RBounds& r_bounds) {
  return maliput::api::RBounds(r_bounds.min, r_bounds.max);
}

maliput_ros_interfaces::msg::SRange ToRosMessage(const maliput::api::SRange& s_range) {
  maliput_ros_interfaces::msg::SRange msg;
  msg.s0 = s_range.s0();
  msg.s1 = s_range.s1();
  msg.size = s_range.size();
  msg.with_s = s_range.WithS();
  return msg;
}

maliput::api::SRange FromRosMessage(const maliput_ros_interfaces::msg::SRange& s_range) {
  return maliput::api::SRange(s_range.s0, s_range.s1);
}

maliput_ros_interfaces::msg::LaneSRange ToRosMessage(const maliput::api::LaneSRange& lane_s_range) {
  maliput_ros_interfaces::msg::LaneSRange msg;
  msg.lane_id = ToRosMessage(lane_s_range.lane_id());
  msg.s_range = ToRosMessage(lane_s_range.s_range());
  return msg;
}

maliput::api::LaneSRange FromRosMessage(const maliput_ros_interfaces::msg::LaneSRange& lane_s_range) {
  return maliput::api::LaneSRange(FromRosMessage(lane_s_range.lane_id), FromRosMessage(lane_s_range.s_range));
}

maliput_ros_interfaces::msg::LaneSRoute ToRosMessage(const maliput::api::LaneSRoute& lane_s_route) {
  maliput_ros_interfaces::msg::LaneSRoute msg;
  msg.ranges.resize(lane_s_route.ranges().size());
  std::transform(lane_s_route.ranges().cbegin(), lane_s_route.ranges().cend(), msg.ranges.begin(),
                 [](const maliput::api::LaneSRange& lane_s_range) { return ToRosMessage(lane_s_range); });
  msg.length = lane_s_route.length();
  return msg;
}

maliput::api::LaneSRoute FromRosMessage(const maliput_ros_interfaces::msg::LaneSRoute& lane_s_route) {
  std::vector<maliput::api::LaneSRange> ranges;
  // Not using std::transform() because we ranges should be resized and it fails to compile due to LaneId
  // LaneSRange.
  for (const auto& range : lane_s_route.ranges) {
    ranges.push_back(FromRosMessage(range));
  }
  return maliput::api::LaneSRoute(ranges);
}

}  // namespace maliput_ros_translation
