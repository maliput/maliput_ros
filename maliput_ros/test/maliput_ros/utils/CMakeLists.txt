ament_add_gtest(yaml_parser_test yaml_parser_test.cc)

macro(add_dependencies_to_test target)
    if (TARGET ${target})

      target_include_directories(${target}
        PRIVATE
          ${PROJECT_SOURCE_DIR}/include
          ${PROJECT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}
          ${PROJECT_SOURCE_DIR}/test
      )

      target_link_libraries(${target}
          maliput::common
          maliput_ros::utils
      )

    endif()
endmacro()

add_dependencies_to_test(yaml_parser_test)
