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
#include <map>
#include <memory>
#include <string>

#include <maliput/plugin/road_network_loader.h>

#include "maliput_ros/ros/maliput_plugin_config_test.h"

namespace maliput_ros {
namespace ros {
namespace test {
namespace {

// Plugin that simply returns the maliput::api::RoadNetwork registered via
// maliput_ros::test::RegisterRoadNetworkForPlugin().
class RoadNetworkLoaderTest : public maliput::plugin::RoadNetworkLoader {
 public:
  std::unique_ptr<maliput::api::RoadNetwork> operator()(const std::map<std::string, std::string>&) const override {
    return GetRoadNetworkForPlugin();
  }

  std::map<std::string, std::string> GetDefaultParameters() const override { return {}; }
};

REGISTER_ROAD_NETWORK_LOADER_PLUGIN("maliput_ros_road_network_plugin", RoadNetworkLoaderTest);

}  // namespace
}  // namespace test
}  // namespace ros
}  // namespace maliput_ros
