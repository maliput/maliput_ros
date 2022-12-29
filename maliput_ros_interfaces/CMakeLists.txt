##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(maliput_ros_interfaces)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

message(STATUS "\n\n========= Project Configuration ========\n")

set(BUILD_SHARED_LIBS true)

##############################################################################
# Sources
##############################################################################

set(MESSAGES
  "msg/BranchPointId.msg"
  "msg/JunctionId.msg"
  "msg/LaneId.msg"
  "msg/RoadGeometryId.msg"
  "msg/SegmentId.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGES}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

##############################################################################
# Tests
##############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

##############################################################################
# Export
##############################################################################
ament_export_dependencies(rosidl_default_runtime)

ament_package()
