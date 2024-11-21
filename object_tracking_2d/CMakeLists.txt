cmake_minimum_required(VERSION 3.8)
project(object_tracking_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(track_msgs REQUIRED)
find_package(ros2_cpp_utils REQUIRED)

include_directories(
  include
  kalman_filter
  hungarian-algorithm-cpp
)

set(dependencies
  rclcpp
  visualization_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  track_msgs
  ros2_cpp_utils
)

add_library(hungarian_lib hungarian-algorithm-cpp/Hungarian.cpp)
target_compile_options(hungarian_lib PUBLIC -Wall)
target_include_directories(hungarian_lib PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hungarian-algorithm-cpp>
  $<INSTALL_INTERFACE:include>
)
ament_export_targets(export_hungarian HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
  hungarian_lib
  EXPORT export_hungarian
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/laser_scan_filter.cpp
  src/object_tracking.cpp
  src/object.cpp
  src/system.cpp
  hungarian-algorithm-cpp/Hungarian.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_include_directories(${PROJECT_NAME}_node PUBLIC
  #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${ros2_cpp_utils_INCLUDE_DIRS}>
)
install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/hungarian-algorithm-cpp  # hungarian-algorithm-cppディレクトリをincludeにインストール
  DESTINATION include
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
