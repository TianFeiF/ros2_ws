cmake_minimum_required(VERSION 3.8)
project(rml_63_servo)

if(CMAKE_COMPILER_IS_GNUCCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rml_63_description REQUIRED)
find_package(rml_63_moveit REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(trajectory_msgs REQUIRED)
find_package(rm_ros_interfaces REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(controller_manager REQUIRED)
# moveit_configs_utils is Python package, no CMake config needed

add_executable(servo_to_real_robot src/servo_to_real_robot.cpp)
ament_target_dependencies(servo_to_real_robot
  rclcpp
  trajectory_msgs
  rm_ros_interfaces
  sensor_msgs
)

install(TARGETS
  servo_to_real_robot
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
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
