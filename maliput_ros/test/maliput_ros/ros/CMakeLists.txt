ament_add_gmock(maliput_query_test maliput_query_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      ament_target_dependencies(${target}
          maliput
      )

      target_link_libraries(${target}
          maliput::test_utilities
          maliput_ros::maliput_ros_node
          maliput_ros_translation::maliput_ros_translation
      )

    endif()
endmacro()

add_dependencies_to_test(maliput_query_test)
