cmake_minimum_required(VERSION 3.8)
project(itajuba_fase1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)           # ROS2 build system and CMake utilities
find_package(fsm REQUIRED)                   # Finite State Machine library for mission control
find_package(itajuba_drone_lib REQUIRED)     # Itajuba drone control library
find_package(eigen3_cmake_module REQUIRED)   # CMake module for finding Eigen3 library
find_package(Eigen3 REQUIRED)                # Linear algebra library for matrix operations
find_package(rclcpp REQUIRED)                # ROS2 C++ client library
find_package(OpenCV 4 REQUIRED)              # Computer vision library for image processing
find_package(vision_msgs REQUIRED)           # Vision-related message types (detections, classifications)
find_package(sensor_msgs REQUIRED)           # Sensor message types (Image, CameraInfo, etc.)
find_package(cv_bridge REQUIRED)             # Bridge between OpenCV and ROS2 image messages
find_package(visualization_msgs REQUIRED)    # Visualization message types for RViz markers

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/../fsm/fsm/include)

#fase1

add_executable(fase1
  src/fase1.cpp
)

ament_target_dependencies(fase1
  rclcpp
  fsm
  itajuba_drone_lib
  Eigen3
  vision_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
)


#fase1_tester
add_executable(fase1_tester
  src/fase1_tester.cpp
)

ament_target_dependencies(fase1_tester
  rclcpp
  fsm
  itajuba_drone_lib
  Eigen3
  vision_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
)

# Link libraries
target_link_libraries(fase1 ${OpenCV_LIBS})
target_link_libraries(fase1_tester ${OpenCV_LIBS})

#Install

install(
  TARGETS
    fase1
    fase1_tester
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
