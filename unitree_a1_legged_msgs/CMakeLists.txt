cmake_minimum_required(VERSION 3.8)
project(unitree_a1_legged_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
    "msg/CommonMotorCmd.msg"
    "msg/ControllerType.msg"
    "msg/ControlMode.msg"
    "msg/FootForceState.msg"
    "msg/HighCmd.msg"
    "msg/HighState.msg"
    "msg/JointCommand.msg"
    "msg/LegCmd.msg"
    "msg/LegState.msg"
    "msg/LowCmd.msg"
    "msg/LowState.msg"
    "msg/MotorCmd.msg"
    "msg/MotorState.msg"
    "msg/QuadrupedCmd.msg"
    "msg/QuadrupedState.msg"
    "msg/UnitreeHeader.msg"
    "srv/Gait.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
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
