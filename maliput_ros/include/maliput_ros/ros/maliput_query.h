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

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/regions.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/api/segment.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/routing/derive_lane_s_routes.h>

namespace maliput_ros {
namespace ros {

/// @brief Proxy to a maliput::api::RoadNetwork to make queries to it.
class MaliputQuery final {
 public:
  struct LaneBoundaries {
    maliput::api::RBounds lane_boundaries{};
    maliput::api::RBounds segment_boundaries{};
    maliput::api::HBounds elevation_boundaries{};
  };

  /// @brief Constructs a MaliputQuery.
  /// @param[in] road_network The maliput::api::RoadNetwork to hold. It must not be nullptr.
  /// @throws maliput::common::assertion_error When @p road_network is nullptr.
  explicit MaliputQuery(std::unique_ptr<maliput::api::RoadNetwork> road_network)
      : road_network_(std::move(road_network)) {
    MALIPUT_THROW_UNLESS(road_network_ != nullptr);
  }

  /// @return The maliput::api::RoadGeometry.
  inline const maliput::api::RoadGeometry* road_geometry() const { return road_network_->road_geometry(); }

  /// Finds a maliput::api::Junction by its ID.
  /// @param[in] id The maliput::api::JunctionId.
  /// @return A maliput::api::Junction when @p id refers to a valid maliput::api::Junction. Otherwise, nullptr.
  inline const maliput::api::Junction* GetJunctionBy(const maliput::api::JunctionId& id) const {
    return road_network_->road_geometry()->ById().GetJunction(id);
  }

  /// Finds a maliput::api::Segment by its ID.
  /// @param[in] id The maliput::api::SegmentId.
  /// @return A maliput::api::Segment when @p id refers to a valid maliput::api::Segment. Otherwise, nullptr.
  inline const maliput::api::Segment* GetSegmentBy(const maliput::api::SegmentId& id) const {
    return road_network_->road_geometry()->ById().GetSegment(id);
  }

  /// Finds a maliput::api::Lane by its ID.
  /// @param[in] id The maliput::api::LaneId.
  /// @return A maliput::api::Lane when @p id refers to a valid maliput::api::Lane. Otherwise, nullptr.
  inline const maliput::api::Lane* GetLaneBy(const maliput::api::LaneId& id) const {
    return road_network_->road_geometry()->ById().GetLane(id);
  }

  /// Finds a maliput::api::BranchPoint by its ID.
  /// @param[in] id The maliput::api::BranchPointId.
  /// @return A maliput::api::BranchPoint when @p id refers to a valid maliput::api::BranchPoint. Otherwise, nullptr.
  inline const maliput::api::BranchPoint* GetBranchPointBy(const maliput::api::BranchPointId& id) const {
    return road_network_->road_geometry()->ById().GetBranchPoint(id);
  }

  /// Converts @p inertial_position into a maliput::api::RoadPostion.
  /// @param[in] inertial_position The maliput::api::InertialPosition to convert into a
  /// maliput::api::RoadPositionResult.
  /// @return A maliput::api::RoadPositionResult that is mapped from @p inertial_position.
  inline const maliput::api::RoadPositionResult ToRoadPosition(
      const maliput::api::InertialPosition& inertial_position) const {
    return road_network_->road_geometry()->ToRoadPosition(inertial_position, std::nullopt);
  }

  /// Finds a list of maliput::api::RoadPositionResults that fall within @p radius distance from @p inertial_position.
  /// @param[in] inertial_position The maliput::api::InertialPosition to  use as a center point.
  /// @param[in] radius The maximum distance from @p inertial_position that the returned values could have.
  /// @return A vector of maliput::api::RoadPositionResult.
  inline const std::vector<maliput::api::RoadPositionResult> FindRoadPositions(
      const maliput::api::InertialPosition& inertial_position, double radius) const {
    return road_network_->road_geometry()->FindRoadPositions(inertial_position, radius);
  }

  /// Computes the INERTIAL Frame position and orientation of a given maliput::api::RoadPosition @p road_position.
  /// @param[in] road_position The maliput::api::RoadPosition to map into the INERTIAL Frame.
  /// @return An optional of a pair holding the maliput::api::InertialPosition and the maliput::api::Rotation at @p
  /// road_position. When the @p road_position.lane is nullptr, it returns std::nullopt.
  inline std::optional<std::pair<maliput::api::InertialPosition, maliput::api::Rotation>> ToInertialPose(
      const maliput::api::RoadPosition& road_position) const {
    if (road_position.lane == nullptr) {
      return {};
    }
    return {std::make_pair(road_position.lane->ToInertialPosition(road_position.pos),
                           road_position.lane->GetOrientation(road_position.pos))};
  }

  /// Computes the motion derivatives at @p road_position scaled by @p velocity.
  /// @param[in] road_position The maliput::api::RoadPosition where to compute the motion derivatives.
  /// @param[in] velocity The maliput::api::IsoLaneVelocity to scale the derivatives.
  /// @return An optional with the motion derivatives packed into a maliput::api::LanePosition.
  /// When the @p road_position.lane is nullptr, it returns std::nullopt.
  inline maliput::api::LanePosition EvalMotionDerivatives(const maliput::api::RoadPosition& road_position,
                                                          const maliput::api::IsoLaneVelocity& velocity) const {
    return road_position.lane == nullptr ? maliput::api::LanePosition{}
                                         : road_position.lane->EvalMotionDerivatives(road_position.pos, velocity);
  }

  /// Computes the lateral and elevation boundaries a given maliput::api::Lane* at @p road_position.
  /// @param[in] road_position The maliput::api::RoadPosition where to compute the boundaries.
  /// @return An optional with a LaneBoundaries struct holding the lane, segment and elevation boundaries.
  /// When the @p road_position.lane is nullptr, it returns std::nullopt.
  inline std::optional<LaneBoundaries> EvalLaneBoundaries(const maliput::api::RoadPosition& road_position) const {
    if (road_position.lane == nullptr) {
      return {};
    }
    const maliput::api::Lane* lane = road_position.lane;
    return {LaneBoundaries{lane->lane_bounds(road_position.pos.s()), lane->segment_bounds(road_position.pos.s()),
                           lane->elevation_bounds(road_position.pos.s(), road_position.pos.r())}};
  }

  /// Computes the different maliput::api::LaneSRoutes that joins @p start to @p end.
  /// @details See maliput::routing::DeriveLaneSRoutes for further details.
  /// @param[in] start The maliput::api::RoadPosition where to start routing.
  /// @param[in] end The maliput::api::RoadPosition where to end routing.
  /// @param[in] max_length_m It is the maximum length of the intermediate lanes
  /// between @p start and @p end.
  /// @return A vector of maliput::api::LaneSRoute. If @p start or @p end have nullptr
  /// as maliput::api::Lanes, or @p max_length_m is negative, or no route has been found
  /// the vector will be empty.
  inline std::vector<maliput::api::LaneSRoute> DeriveLaneSRoutes(const maliput::api::RoadPosition& start,
                                                                 const maliput::api::RoadPosition& end,
                                                                 double max_length_m) const {
    if (start.lane == nullptr || end.lane == nullptr || max_length_m < 0.) {
      return {};
    }
    return maliput::routing::DeriveLaneSRoutes(start, end, max_length_m);
  }

  /// Samples in the LANE Frame the @p lane_s_route at constant @p path_length_sampling_rate.
  /// @param[in] lane_s_route The maliput::api::LaneSRoute to sample.
  /// @param[in] path_length_sampling_rate The distance in LANE Frame between samples along the centerline of @p
  /// lane_s_route. It must be positive.
  /// @return A vector of maliput::api::InertialPosition with the sampled waypoints. When @p path_length_sampling_rate
  /// is negative, it returns an empty vector.
  inline std::vector<maliput::api::InertialPosition> SampleAheadWaypoints(const maliput::api::LaneSRoute& lane_s_route,
                                                                          double path_length_sampling_rate) const {
    return path_length_sampling_rate <= 0.
               ? std::vector<maliput::api::InertialPosition>{}
               : road_network_->road_geometry()->SampleAheadWaypoints(lane_s_route, path_length_sampling_rate);
  }

 private:
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{};
};

}  // namespace ros
}  // namespace maliput_ros
