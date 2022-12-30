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
#include "maliput_ros/utils/yaml_parser.h"

#include <maliput/common/maliput_throw.h>
#include <yaml-cpp/yaml.h>

namespace maliput_ros {
namespace utils {

MaliputRoadNetworkConfiguration LoadYamlConfigFile(const std::string& file_path) {
  static constexpr char kMaliputKey[] = "maliput";
  static constexpr char kBackendKey[] = "backend";
  static constexpr char kParametersKey[] = "parameters";
  YAML::Node root = YAML::LoadFile(file_path);
  const auto maliput_node = root[kMaliputKey];
  MALIPUT_VALIDATE(maliput_node.IsDefined(), "maliput node not found in config file");
  MALIPUT_VALIDATE(maliput_node.IsMap(), "maliput node is not a map");
  const auto maliput_backend_node = maliput_node[kBackendKey];
  MALIPUT_VALIDATE(maliput_backend_node.IsDefined(), "backend node not found in config file");

  std::map<std::string, std::string> parameters;
  const auto parameters_node = maliput_node[kParametersKey];
  if (parameters_node.IsDefined()) {
    // Note that if it isn't defined then we just leave the parameters map empty.
    MALIPUT_VALIDATE(parameters_node.IsMap(), "parameters node is not a map");
  }
  for (const auto& param : parameters_node) {
    parameters[param.first.as<std::string>()] = param.second.as<std::string>();
  }
  return MaliputRoadNetworkConfiguration{maliput_backend_node.as<std::string>(), parameters};
}

}  // namespace utils
}  // namespace maliput_ros
