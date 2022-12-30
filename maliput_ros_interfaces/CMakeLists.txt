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
find_package(geometry_msgs REQUIRED)
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
  "msg/BranchPoint.msg"
  "msg/BranchPointId.msg"
  "msg/Junction.msg"
  "msg/JunctionId.msg"
  "msg/Lane.msg"
  "msg/LaneEnd.msg"
  "msg/LaneEndSet.msg"
  "msg/LaneId.msg"
  "msg/RoadGeometry.msg"
  "msg/RoadGeometryId.msg"
  "msg/Segment.msg"
  "msg/SegmentId.msg"
)

set(SERVICE_MSGS
  "srv/BranchPoint.srv"
  "srv/Junction.srv"
  "srv/Lane.srv"
  "srv/RoadGeometry.srv"
  "srv/Segment.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGES}
  ${SERVICE_MSGS}
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
