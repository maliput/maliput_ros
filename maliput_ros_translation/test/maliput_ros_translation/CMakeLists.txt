ament_add_gmock(convert_test convert_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      ament_target_dependencies(${target}
          builtin_interfaces
          geometry_msgs
          maliput
          maliput_ros_interfaces
      )

      target_link_libraries(${target}
          maliput::test_utilities
          maliput_ros_translation::maliput_ros_translation
      )

    endif()
endmacro()

add_dependencies_to_test(convert_test)
