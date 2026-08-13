cmake_minimum_required(VERSION 3.8)
project(volleyball_pid_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(volleyball_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(volleyball_detect REQUIRED)

# detector.hpp 包含 librealsense2/rs.hpp 和 openvino/openvino.hpp
find_package(OpenVINO REQUIRED)

# 从 openvino::runtime 目标获取 include 路径
get_target_property(OPENVINO_INCLUDE_DIRS openvino::runtime INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "OpenVINO include dirs: ${OPENVINO_INCLUDE_DIRS}")

include_directories(include)

add_executable(pid_camera_node
    src/pid_camera_detector.cpp
    src/pid_camera_main.cpp
    src/pid_camera_node.cpp)

ament_target_dependencies(pid_camera_node
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV
    ament_index_cpp
    volleyball_interfaces
    volleyball_detect
    image_transport
)

# detector.hpp 包含 librealsense2/rs.hpp
target_include_directories(pid_camera_node SYSTEM PRIVATE
  ${OPENVINO_INCLUDE_DIRS}
)

# 链接 volleyball_detect 共享库及其传递依赖
target_link_libraries(pid_camera_node
    openvino::runtime
)

install(TARGETS pid_camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
