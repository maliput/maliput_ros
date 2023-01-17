# It is necessary to find gmock via this macro so GMOCK_INCLUDE_DIRS and GMOCK_LIBRARIES are populated.
# That way, we can use gmock linked to libraries that will be used by test executables later on.
ament_find_gmock()

add_library(maliput_ros_plugin_config_test
    maliput_plugin_config_test.cc
)

add_library(maliput_ros::maliput_ros_plugin_config_test ALIAS maliput_ros_plugin_config_test)

set_target_properties(maliput_ros_plugin_config_test
  PROPERTIES
    OUTPUT_NAME maliput_ros_plugin_config_test
)

target_link_libraries(maliput_ros_plugin_config_test
    maliput::api
    maliput::plugin
    ${GMOCK_LIBRARIES}
)

target_include_directories(maliput_ros_plugin_config_test
  PRIVATE
    ${PROJECT_SOURCE_DIR}/test
    "${GMOCK_INCLUDE_DIRS}"
)

add_library(maliput_ros_road_network_plugin
    maliput_ros_road_network_plugin.cc
)

add_library(maliput_ros::maliput_ros_road_network_plugin ALIAS maliput_ros_road_network_plugin)

set_target_properties(maliput_ros_road_network_plugin
  PROPERTIES
    OUTPUT_NAME maliput_ros_road_network_plugin
)

target_link_libraries(maliput_ros_road_network_plugin
    maliput::api
    maliput::plugin
    maliput_ros_plugin_config_test
    ${GMOCK_LIBRARIES}
)

target_include_directories(maliput_ros_road_network_plugin
  PRIVATE
    ${PROJECT_SOURCE_DIR}/test
    "${GMOCK_INCLUDE_DIRS}"
)

set(TEST_PLUGIN_INSTALL_DIR "/tmp/maliput_ros/test/plugins/")
set(TEST_YAML_CONFIGURATION_PLUGIN_INSTALL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test_maliput_plugin.yml")

install(
  TARGETS maliput_ros_road_network_plugin
  ARCHIVE DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
  LIBRARY DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
  RUNTIME DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
)

ament_add_gmock(branch_point_service_test branch_point_service_test.cc)
ament_add_gmock(junction_service_test junction_service_test.cc)
ament_add_gmock(lane_service_test lane_service_test.cc)
ament_add_gmock(maliput_query_test maliput_query_test.cc)
ament_add_gmock(maliput_query_node_test maliput_query_node_test.cc)
ament_add_gmock(road_geometry_service_test road_geometry_service_test.cc)
ament_add_gmock(segment_service_test segment_service_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      ament_target_dependencies(${target}
          lifecycle_msgs
          maliput
          rclcpp
          rclcpp_lifecycle
      )

      target_link_libraries(${target}
          maliput::plugin
          maliput::test_utilities
          maliput_ros::maliput_ros_node
          maliput_ros::maliput_ros_plugin_config_test
          maliput_ros_translation::maliput_ros_translation
      )

      target_compile_definitions(${target}
        PRIVATE
          TEST_MALIPUT_PLUGIN_LIBDIR="${TEST_PLUGIN_INSTALL_DIR}"
          TEST_YAML_CONFIGURATION_PLUGIN_INSTALL_PATH="${TEST_YAML_CONFIGURATION_PLUGIN_INSTALL_PATH}"
      )
    endif()
endmacro()

add_dependencies_to_test(branch_point_service_test)
add_dependencies_to_test(junction_service_test)
add_dependencies_to_test(lane_service_test)
add_dependencies_to_test(maliput_query_test)
add_dependencies_to_test(maliput_query_node_test)
add_dependencies_to_test(road_geometry_service_test)
add_dependencies_to_test(segment_service_test)
