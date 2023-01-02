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
#include "maliput_ros/ros/maliput_query.h"

#include <optional>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <maliput/api/road_network.h>
#include <maliput/common/assertion_error.h>

#include "maliput_ros/ros/maliput_mock.h"

namespace maliput_ros {
namespace ros {
namespace test {
namespace {

class MaliputQueryTest : public ::testing::Test {
 public:
  void SetUp() override {
    auto road_geometry = std::make_unique<RoadGeometryMock>();
    road_geometry_ptr_ = road_geometry.get();
    auto road_rulebook = std::make_unique<RoadRulebookMock>();
    road_rulebook_ptr_ = road_rulebook.get();
    auto traffic_light_book = std::make_unique<TrafficLightBookMock>();
    traffic_light_book_ptr_ = traffic_light_book.get();
    auto intersection_book = std::make_unique<IntersectionBookMock>();
    intersection_book_ptr_ = intersection_book.get();
    auto phase_ring_book = std::make_unique<PhaseRingBookMock>();
    phase_ring_book_ptr_ = phase_ring_book.get();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    auto right_of_way_rule_state_provider = std::make_unique<RightOfWayRuleStateProviderMock>();
    right_of_way_rule_state_provider_ptr_ = right_of_way_rule_state_provider.get();
#pragma GCC diagnostic pop
    auto phase_provider = std::make_unique<PhaseProviderMock>();
    phase_provider_ptr_ = phase_provider.get();
    auto discrete_value_rule_state_provider = std::make_unique<DiscreteValueRuleStateProviderMock>();
    discrete_value_rule_state_provider_ptr_ = discrete_value_rule_state_provider.get();
    auto range_value_rule_state_provider = std::make_unique<RangeValueRuleStateProviderMock>();
    range_value_rule_state_provider_ptr_ = range_value_rule_state_provider.get();

    auto road_network = std::make_unique<maliput::api::RoadNetwork>(
        std::move(road_geometry), std::move(road_rulebook), std::move(traffic_light_book), std::move(intersection_book),
        std::move(phase_ring_book), std::move(right_of_way_rule_state_provider), std::move(phase_provider),
        std::make_unique<maliput::api::rules::RuleRegistry>(), std::move(discrete_value_rule_state_provider),
        std::move(range_value_rule_state_provider));

    dut_ = std::make_unique<MaliputQuery>(std::move(road_network));
  }

  RoadGeometryMock* road_geometry_ptr_{};
  RoadRulebookMock* road_rulebook_ptr_{};
  TrafficLightBookMock* traffic_light_book_ptr_{};
  IntersectionBookMock* intersection_book_ptr_{};
  PhaseRingBookMock* phase_ring_book_ptr_{};
  RightOfWayRuleStateProviderMock* right_of_way_rule_state_provider_ptr_{};
  PhaseProviderMock* phase_provider_ptr_{};
  DiscreteValueRuleStateProviderMock* discrete_value_rule_state_provider_ptr_{};
  RangeValueRuleStateProviderMock* range_value_rule_state_provider_ptr_{};
  std::unique_ptr<MaliputQuery> dut_;
};

TEST_F(MaliputQueryTest, ConstructorValidation) {
  std::unique_ptr<maliput::api::RoadNetwork> road_network{};
  ASSERT_THROW({ MaliputQuery(std::move(road_network)); }, maliput::common::assertion_error);
}

TEST_F(MaliputQueryTest, RoadGeometry) {
  ASSERT_EQ(dut_->road_geometry(), static_cast<const maliput::api::RoadGeometry*>(road_geometry_ptr_));
}

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
