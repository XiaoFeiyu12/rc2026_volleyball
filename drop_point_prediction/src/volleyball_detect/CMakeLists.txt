cmake_minimum_required(VERSION 3.8)
project(volleyball_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(volleyball_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(realsense2 REQUIRED)
find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

install(DIRECTORY include
DESTINATION include/${PROJECT_NAME})

add_executable(volleyball_detect
  src/detector_main.cpp
  src/detector_node.cpp
  src/detector.cpp)

ament_target_dependencies(volleyball_detect
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  OpenCV
  cv_bridge
  Eigen3
  realsense2_camera_msgs
  OpenVINO
  volleyball_interfaces
  ament_index_cpp
  message_filters
  image_transport
)

target_link_libraries(volleyball_detect
    openvino::runtime
    ${realsense2_LIBRARY}
)

# librealsense2 (realsense2) 的 CMake 包导出的是 realsense2_INCLUDE_DIR（单数），
# ament_target_dependencies 不会自动把它加到编译 include path 里，所以这里显式添加。
target_include_directories(volleyball_detect SYSTEM PRIVATE
  ${realsense2_INCLUDE_DIR}
)

install(DIRECTORY
  model
  DESTINATION share/${PROJECT_NAME})

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

install(TARGETS volleyball_detect
  DESTINATION lib/${PROJECT_NAME})
