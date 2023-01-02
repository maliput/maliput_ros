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

#include <optional>
#include <vector>

#include <gmock/gmock.h>
#include <maliput/api/intersection_book.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/regions.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/road_network.h>
#include <maliput/api/rules/discrete_value_rule_state_provider.h>
#include <maliput/api/rules/phase_provider.h>
#include <maliput/api/rules/phase_ring_book.h>
#include <maliput/api/rules/range_value_rule_state_provider.h>
#include <maliput/api/rules/right_of_way_rule_state_provider.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/api/rules/rule_registry.h>
#include <maliput/api/rules/traffic_light_book.h>

namespace maliput_ros {
namespace ros {
namespace test {

class RoadGeometryMock final : public maliput::api::RoadGeometry {
 public:
  MOCK_CONST_METHOD0(do_id, maliput::api::RoadGeometryId());
  MOCK_CONST_METHOD0(do_num_junctions, int());
  MOCK_CONST_METHOD1(do_junction, const maliput::api::Junction*(int));
  MOCK_CONST_METHOD0(do_num_branch_points, int());
  MOCK_CONST_METHOD1(do_branch_point, const maliput::api::BranchPoint*(int));
  MOCK_CONST_METHOD0(DoById, const maliput::api::RoadGeometry::IdIndex&());
  MOCK_CONST_METHOD2(DoToRoadPosition,
                     maliput::api::RoadPositionResult(const maliput::api::InertialPosition&,
                                                      const std::optional<maliput::api::RoadPosition>&));
  MOCK_CONST_METHOD2(DoFindRoadPositions,
                     std::vector<maliput::api::RoadPositionResult>(const maliput::api::InertialPosition&, double));
  MOCK_CONST_METHOD0(do_linear_tolerance, double());
  MOCK_CONST_METHOD0(do_angular_tolerance, double());
  MOCK_CONST_METHOD0(do_scale_length, double());
  MOCK_CONST_METHOD2(DoSampleAheadWaypoints,
                     std::vector<maliput::api::InertialPosition>(const maliput::api::LaneSRoute&, double));
  MOCK_CONST_METHOD0(do_inertial_to_backend_frame_translation, maliput::math::Vector3());
};

class RoadRulebookMock final : public maliput::api::rules::RoadRulebook {
 public:
  MOCK_CONST_METHOD2(DoFindRules,
                     maliput::api::rules::RoadRulebook::QueryResults(const std::vector<maliput::api::LaneSRange>&,
                                                                     double));
  MOCK_CONST_METHOD0(DoRules, maliput::api::rules::RoadRulebook::QueryResults());
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_CONST_METHOD1(DoGetRule, maliput::api::rules::RightOfWayRule(const maliput::api::rules::RightOfWayRule::Id&));
  MOCK_CONST_METHOD1(DoGetRule, maliput::api::rules::SpeedLimitRule(const maliput::api::rules::SpeedLimitRule::Id&));
  MOCK_CONST_METHOD1(DoGetRule,
                     maliput::api::rules::DirectionUsageRule(const maliput::api::rules::DirectionUsageRule::Id&));
#pragma GCC diagnostic pop
  MOCK_CONST_METHOD1(DoGetDiscreteValueRule,
                     maliput::api::rules::DiscreteValueRule(const maliput::api::rules::Rule::Id&));
  MOCK_CONST_METHOD1(DoGetRangeValueRule, maliput::api::rules::RangeValueRule(const maliput::api::rules::Rule::Id&));
};

class TrafficLightBookMock final : public maliput::api::rules::TrafficLightBook {
 public:
  MOCK_CONST_METHOD1(DoGetTrafficLight,
                     const maliput::api::rules::TrafficLight*(const maliput::api::rules::TrafficLight::Id&));
  MOCK_CONST_METHOD0(DoTrafficLights, std::vector<const maliput::api::rules::TrafficLight*>());
};

class IntersectionBookMock final : public maliput::api::IntersectionBook {
 public:
  MOCK_METHOD0(DoGetIntersections, std::vector<maliput::api::Intersection*>());
  MOCK_METHOD1(DoGetIntersection, maliput::api::Intersection*(const maliput::api::Intersection::Id&));
  MOCK_METHOD1(DoGetFindIntersection, maliput::api::Intersection*(const maliput::api::rules::TrafficLight::Id&));
  MOCK_METHOD1(DoGetFindIntersection, maliput::api::Intersection*(const maliput::api::rules::DiscreteValueRule::Id&));
  MOCK_METHOD1(DoGetFindIntersection, maliput::api::Intersection*(const maliput::api::InertialPosition&));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_METHOD1(DoGetFindIntersection, maliput::api::Intersection*(const maliput::api::rules::RightOfWayRule::Id&));
#pragma GCC diagnostic pop
};

class PhaseRingBookMock : public maliput::api::rules::PhaseRingBook {
 public:
  MOCK_CONST_METHOD0(DoGetPhaseRings, std::vector<maliput::api::rules::PhaseRing::Id>());
  MOCK_CONST_METHOD1(DoGetPhaseRing,
                     std::optional<maliput::api::rules::PhaseRing>(const maliput::api::rules::PhaseRing::Id&));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MOCK_CONST_METHOD1(DoFindPhaseRing,
                     std::optional<maliput::api::rules::PhaseRing>(const maliput::api::rules::RightOfWayRule::Id&));
#pragma GCC diagnostic pop
  MOCK_CONST_METHOD1(DoFindPhaseRing,
                     std::optional<maliput::api::rules::PhaseRing>(const maliput::api::rules::Rule::Id&));
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
class RightOfWayRuleStateProviderMock final : public maliput::api::rules::RightOfWayRuleStateProvider {
 public:
  MOCK_CONST_METHOD1(DoGetState, std::optional<maliput::api::rules::RightOfWayRuleStateProvider::RightOfWayResult>(
                                     const maliput::api::rules::RightOfWayRule::Id&));
};
#pragma GCC diagnostic pop

class PhaseProviderMock final : public maliput::api::rules::PhaseProvider {
 public:
  MOCK_CONST_METHOD1(
      DoGetPhase, std::optional<maliput::api::rules::PhaseProvider::Result>(const maliput::api::rules::PhaseRing::Id&));
};

class DiscreteValueRuleStateProviderMock final : public maliput::api::rules::DiscreteValueRuleStateProvider {
 public:
  MOCK_CONST_METHOD1(DoGetState, std::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult>(
                                     const maliput::api::rules::Rule::Id&));
  MOCK_CONST_METHOD3(DoGetState,
                     std::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult>(
                         const maliput::api::RoadPosition&, const maliput::api::rules::Rule::TypeId&, double));
};

class RangeValueRuleStateProviderMock final : public maliput::api::rules::RangeValueRuleStateProvider {
 public:
  MOCK_CONST_METHOD1(DoGetState, std::optional<maliput::api::rules::RangeValueRuleStateProvider::StateResult>(
                                     const maliput::api::rules::Rule::Id&));
  MOCK_CONST_METHOD3(DoGetState,
                     std::optional<maliput::api::rules::RangeValueRuleStateProvider::StateResult>(
                         const maliput::api::RoadPosition&, const maliput::api::rules::Rule::TypeId&, double));
};

}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
