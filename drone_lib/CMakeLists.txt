cmake_minimum_required(VERSION 3.8)
project(itajuba_drone_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)           # ROS2 build system and CMake utilities
find_package(eigen3_cmake_module REQUIRED)   # CMake module for finding Eigen3 library
find_package(Eigen3 REQUIRED)                # Linear algebra library for matrix operations
find_package(px4_msgs REQUIRED)              # PX4 autopilot message definitions
find_package(tf2 REQUIRED)                   # Transform library for coordinate frame conversions
find_package(rclcpp REQUIRED)                # ROS2 C++ client library
find_package(cv_bridge REQUIRED)             # Bridge between OpenCV and ROS2 image messages
find_package(custom_msgs REQUIRED)           # Custom message definitions for the project
find_package(geometry_msgs REQUIRED)         # Standard geometry message types (Point, Pose, Twist, etc.)
find_package(nav_msgs REQUIRED)              # Navigation message types (Odometry, Path, etc.)



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



include_directories(include)

add_library(itajuba_drone_lib SHARED
  src/Drone.cpp
)

ament_target_dependencies(itajuba_drone_lib
  Eigen3
  px4_msgs
  rclcpp
  tf2
  cv_bridge
  custom_msgs
)

add_executable(system_health
  src/system_health.cpp
)

ament_target_dependencies(system_health
  rclcpp
  custom_msgs
)

ament_export_targets(itajuba_drone_lib HAS_LIBRARY_TARGET)

ament_export_dependencies(Eigen3 px4_msgs rclcpp tf2 cv_bridge custom_msgs)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/drone
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
 
install(
  TARGETS itajuba_drone_lib
  EXPORT itajuba_drone_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS system_health
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
