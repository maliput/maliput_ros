##############################################################################
# Sources
##############################################################################

add_library(maliput_ros_node
  maliput_query_node.cc
)

add_library(maliput_ros::maliput_ros_node ALIAS maliput_ros_node)

set_target_properties(maliput_ros_node
  PROPERTIES
    OUTPUT_NAME maliput_ros_node
)

target_include_directories(maliput_ros_node
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(maliput_ros_node
  maliput
  maliput_ros_interfaces
  maliput_ros_translation
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(maliput_ros_node
  maliput_ros::utils
)

add_executable(maliput_ros
  maliput_query_server.cc
)

target_include_directories(maliput_ros
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(maliput_ros
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(maliput_ros
  maliput_ros::maliput_ros_node
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS maliput_ros_node
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS maliput_ros
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION lib/${PROJECT_NAME}
)
