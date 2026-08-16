cmake_minimum_required(VERSION 3.8)
project(volleyball_ibvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(volleyball_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(ibvs_controller
    src/ibvs_controller_node.cpp
    src/ibvs_main.cpp)

ament_target_dependencies(ibvs_controller
    rclcpp
    volleyball_interfaces
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs)

install(TARGETS ibvs_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
