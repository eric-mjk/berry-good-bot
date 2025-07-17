cmake_minimum_required(VERSION 3.8)
project(strawberry_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/fake_nodes.cpp                                 # ← 새 파일 추가

)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp_v3 rclcpp pluginlib)

# 플러그인 내보내기 (ROS 2 Tut.)  :contentReference[oaicite:6]{index=6}
pluginlib_export_plugin_description_file(
  behaviortree_cpp_v3 strawberry_bt_nodes_plugins.xml)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
