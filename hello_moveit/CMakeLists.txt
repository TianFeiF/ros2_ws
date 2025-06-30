cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KeyboardState.msg"
  DEPENDENCIES builtin_interfaces
)

# Add hello_moveit demo executable
add_executable(hello_moveit_demo src/hello_moveit.cpp)
target_include_directories(hello_moveit_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit_demo
  "moveit_ros_planning_interface"
  "rclcpp"
  "moveit_visual_tools"
)

# Add keyboard publisher executable
add_executable(keyboard_publisher src/keyboard_publisher.cpp)
ament_target_dependencies(keyboard_publisher 
  "rclcpp"
  "std_msgs"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(keyboard_publisher "${cpp_typesupport_target}")

# Add moveit subscriber executable  
add_executable(moveit_subscriber src/moveit_subscriber.cpp)
ament_target_dependencies(moveit_subscriber 
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)
rosidl_get_typesupport_target(cpp_typesupport_target2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(moveit_subscriber "${cpp_typesupport_target2}")

# Install targets
install(TARGETS
  hello_moveit_demo
  keyboard_publisher
  moveit_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(moveit_visual_tools REQUIRED)
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
