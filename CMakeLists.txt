cmake_minimum_required(VERSION 3.5)

project(optical_flow_node)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include PX4-OpticalFlow headers
include_directories(src)
include_directories(${CMAKE_SOURCE_DIR}/../PX4-OpticalFlow/include)

include_directories(${CMAKE_SOURCE_DIR}/../PX4-OpticalFlow/external/klt_feature_tracker/include)

set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})


add_executable(optical_flow_node src/main.cpp)

target_link_libraries(optical_flow_node
  ${OpenCV_LIBS}
)


include_directories(${OpenCV_INCLUDE_DIRS})


ament_target_dependencies(optical_flow_node tf2 tf2_ros tf2_geometry_msgs image_transport rclcpp std_msgs mavros_msgs geometry_msgs sensor_msgs nav_msgs cv_bridge OpenCV)

# Link PX4-OpticalFlow library
target_link_libraries(optical_flow_node
  ${CMAKE_SOURCE_DIR}/../PX4-OpticalFlow/build/OpticalFlow/libOpticalFlow.so
)


install(TARGETS
  optical_flow_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
