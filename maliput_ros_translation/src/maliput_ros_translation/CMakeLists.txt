##############################################################################
# Sources
##############################################################################

add_library(maliput_ros_translation
  convert.cc
)

add_library(maliput_ros_translation::maliput_ros_translation ALIAS maliput_ros_translation)

set_target_properties(maliput_ros_translation
  PROPERTIES
    OUTPUT_NAME maliput_ros_translation
)

target_include_directories(maliput_ros_translation
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(maliput_ros_translation
  builtin_interfaces
  geometry_msgs
  maliput
  maliput_ros_interfaces
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS maliput_ros_translation
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
