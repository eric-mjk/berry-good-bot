cmake_minimum_required(VERSION 3.8)
project(berry_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # 빌드 타임


# ⚠️ 인터페이스 정의
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToNamedPose.action"
  "action/MoveToPose.action"
  "action/ServoTwist.action"
  "action/DetectStrawberry.action"
  "action/GripperControl.action"
  "action/VisualServoDetect.action"
  "msg/DetectionBBox.msg"
  DEPENDENCIES std_msgs geometry_msgs          # 사용하는 메시지들
)

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
