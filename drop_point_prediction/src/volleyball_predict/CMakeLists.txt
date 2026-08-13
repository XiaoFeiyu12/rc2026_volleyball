cmake_minimum_required(VERSION 3.8)
project(volleyball_predict)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(volleyball_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(volleyball_predict 
    src/predictor_main.cpp
    src/predictor.cpp
    src/predictor_node.cpp
)

ament_target_dependencies( volleyball_predict
  rclcpp
  geometry_msgs
  Eigen3
  volleyball_interfaces
  visualization_msgs
)

install(TARGETS
  volleyball_predict
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
