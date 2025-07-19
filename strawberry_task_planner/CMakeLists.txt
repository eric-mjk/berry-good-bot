cmake_minimum_required(VERSION 3.8)
project(strawberry_task_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

find_package(ament_index_cpp REQUIRED) # 추가


# 모든 실행 파일이 사용할 의존성 목록을 정의합니다. (ros2_bt_tut 스타일)
set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  behaviortree_cpp_v3
  ament_index_cpp
)

# --- 이 부분이 중요합니다! ---
# 패키지의 include 디렉토리를 전역적으로 추가합니다.
# CMakeLists.txt 파일이 있는 디렉토리(${CMAKE_CURRENT_SOURCE_DIR})에서 'include' 폴더의 절대 경로를 만듭니다.
# 이렇게 하면 컴파일러가 'include/' 아래의 모든 서브폴더를 탐색합니다.
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# 실행 파일을 정의합니다. (이 명령이 include_directories 뒤에 와야 합니다.)
add_executable(run_bt_node src/run_bt_node.cpp)

# 실행 파일에 의존성을 연결합니다.
ament_target_dependencies(
  run_bt_node
  ${dependencies}
)

# 실행 파일 설치
install(TARGETS run_bt_node DESTINATION lib/${PROJECT_NAME})


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
