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
#include <utility>

#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/api/segment.h>
#include <maliput/common/maliput_throw.h>

namespace maliput_ros {
namespace ros {

/// @brief Proxy to a maliput::api::RoadNetwork to make queries to it.
class MaliputQuery final {
 public:
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

 private:
  std::unique_ptr<maliput::api::RoadNetwork> road_network_{};
};

}  // namespace ros
}  // namespace maliput_ros
