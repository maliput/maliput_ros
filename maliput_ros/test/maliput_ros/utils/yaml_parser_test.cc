// BSD 3-Clause License
//
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

#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/common/filesystem.h>
#include <maliput/common/maliput_throw.h>

namespace maliput_ros {
namespace utils {
namespace tests {
namespace {

class YamlParserTest : public ::testing::Test {
 public:
  static constexpr char kYamlParserTestFolder[]{"yaml_parser_test"};
  static constexpr char kYamlDocumentFileName[]{"document.yaml"};

  void SetUp() override {
    base_path_.set_as_temp();
    base_path_.append(kYamlParserTestFolder);
    ASSERT_TRUE(maliput::common::Filesystem::create_directory_recursive(base_path_));
    yaml_path_.set_path(base_path_.get_path() + "/" + kYamlDocumentFileName);
  }

  void TearDown() override {
    if (!yaml_path_.get_path().empty()) {
      ASSERT_TRUE(maliput::common::Filesystem::remove_file(yaml_path_));
    }
    ASSERT_TRUE(maliput::common::Filesystem::remove_directory(base_path_));
  }

  // Creates a file in `yaml_path_` filled with `content`.
  // @param content String content of the file.
  void CreateFileAndWriteContents(const std::string& content) {
    std::ofstream ofs{yaml_path_.get_path()};
    ofs << content;
    ofs.flush();
    ofs.close();
  }

  maliput::common::Path base_path_;
  maliput::common::Path yaml_path_;
};

TEST_F(YamlParserTest, CorrectParsingOfTheConfiguration) {
  static constexpr const char* kGoodYamlDocument = R"R(
maliput:
  backend: "backend_name"
  parameters:
    prop1: "value1"
    prop2: "value2"
)R";
  CreateFileAndWriteContents(kGoodYamlDocument);

  const MaliputRoadNetworkConfiguration configuration = LoadYamlConfigFile(yaml_path_.get_path());

  ASSERT_EQ(configuration.backend_name, "backend_name");
  ASSERT_EQ(configuration.backend_parameters.size(), static_cast<size_t>(2));
  ASSERT_EQ(configuration.backend_parameters.at("prop1"), "value1");
  ASSERT_EQ(configuration.backend_parameters.at("prop2"), "value2");
}

TEST_F(YamlParserTest, CorrectParsingWhenEmptyProperties) {
  static constexpr const char* kMissingParametersKeyYamlDocument = R"R(
maliput:
  backend: "backend_name"
)R";
  CreateFileAndWriteContents(kMissingParametersKeyYamlDocument);

  const MaliputRoadNetworkConfiguration configuration = LoadYamlConfigFile(yaml_path_.get_path());

  ASSERT_EQ(configuration.backend_name, "backend_name");
  ASSERT_TRUE(configuration.backend_parameters.empty());
}

TEST_F(YamlParserTest, WrongMaliputKeyInDocumentThrows) {
  static constexpr const char* kWrongMaliputKeyYamlDocument = R"R(
not_a_maliput_key:
  backend: "backend_name"
  parameters:
    prop1: "value1"
    prop2: "value2"
)R";
  CreateFileAndWriteContents(kWrongMaliputKeyYamlDocument);

  ASSERT_THROW({ LoadYamlConfigFile(yaml_path_.get_path()); }, maliput::common::assertion_error);
}

TEST_F(YamlParserTest, MissingBackendKeyInDocumentThrows) {
  static constexpr const char* kMissingBackendKeyYamlDocument = R"R(
maliput:
  parameters:
    prop1: "value1"
    prop2: "value2"
)R";
  CreateFileAndWriteContents(kMissingBackendKeyYamlDocument);

  ASSERT_THROW({ LoadYamlConfigFile(yaml_path_.get_path()); }, maliput::common::assertion_error);
}

TEST_F(YamlParserTest, ParameterIsNotAMapDocumentThrows) {
  static constexpr const char* kParameterIsNotAMapYamlDocument = R"R(
maliput:
  backend: "backend_name"
  parameters: "any_parameter"
)R";
  CreateFileAndWriteContents(kParameterIsNotAMapYamlDocument);

  ASSERT_THROW({ LoadYamlConfigFile(yaml_path_.get_path()); }, maliput::common::assertion_error);
}

}  // namespace
}  // namespace tests
}  // namespace utils
}  // namespace maliput_ros
