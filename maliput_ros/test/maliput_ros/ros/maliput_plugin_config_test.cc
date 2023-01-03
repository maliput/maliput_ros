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
#include "maliput_ros/ros/maliput_plugin_config_test.h"

#include <utility>

namespace maliput_ros {
namespace ros {
namespace test {
namespace {

RoadNetworkMockPointers road_network_ptrs_{};
std::unique_ptr<maliput::api::RoadNetwork> road_network_{};

}  // namespace

RoadNetworkMockPointers GetRoadNetworkMockPointers() { return road_network_ptrs_; }

void RegisterRoadNetworkForPlugin(std::unique_ptr<maliput::api::RoadNetwork> road_network) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  road_network_ptrs_ = RoadNetworkMockPointers{
      dynamic_cast<RoadGeometryMock*>(const_cast<maliput::api::RoadGeometry*>(road_network->road_geometry())),
      dynamic_cast<RoadRulebookMock*>(const_cast<maliput::api::rules::RoadRulebook*>(road_network->rulebook())),
      dynamic_cast<TrafficLightBookMock*>(
          const_cast<maliput::api::rules::TrafficLightBook*>(road_network->traffic_light_book())),
      dynamic_cast<IntersectionBookMock*>(road_network->intersection_book()),
      dynamic_cast<PhaseRingBookMock*>(
          const_cast<maliput::api::rules::PhaseRingBook*>(road_network->phase_ring_book())),
      dynamic_cast<RightOfWayRuleStateProviderMock*>(road_network->right_of_way_rule_state_provider()),
      dynamic_cast<PhaseProviderMock*>(road_network->phase_provider()),
      dynamic_cast<DiscreteValueRuleStateProviderMock*>(road_network->discrete_value_rule_state_provider()),
      dynamic_cast<RangeValueRuleStateProviderMock*>(road_network->range_value_rule_state_provider()),
  };
#pragma GCC diagnostic pop
  road_network_ = std::move(road_network);
}

std::unique_ptr<maliput::api::RoadNetwork> GetRoadNetworkForPlugin() { return std::move(road_network_); }

}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
