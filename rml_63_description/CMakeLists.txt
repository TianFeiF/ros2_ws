cmake_minimum_required(VERSION 3.8)

project(rml_63_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


install(DIRECTORY
	launch
	urdf
	meshes
	config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
