cmake_minimum_required(VERSION 3.8)

project(volleyball_robot_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
	urdf 
	config
	DESTINATION share/${PROJECT_NAME})

ament_package()
