add_library(utils
  yaml_parser.cc
)

add_library(maliput_ros::utils ALIAS utils)
set_target_properties(utils
  PROPERTIES
    OUTPUT_NAME maliput_ros_utils
)

target_link_libraries(utils
  maliput::common
  yaml-cpp
)

target_include_directories(utils
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)

install(
  TARGETS utils
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
