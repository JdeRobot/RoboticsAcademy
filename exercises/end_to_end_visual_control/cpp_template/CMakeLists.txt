cmake_minimum_required(VERSION 3.8)
project(academy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(common_interfaces_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_path(ONNXRUNTIME_INCLUDE_DIR NAMES onnxruntime_cxx_api.h)
find_library(ONNXRUNTIME_LIB NAMES onnxruntime)

set(dependencies
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  gazebo_msgs
  message_filters
  tf2_ros
  tf2_geometry_msgs
  common_interfaces_cpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/include
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/include/vacuum_cleaner_cpp
  ${ONNXRUNTIME_INCLUDE_DIR}
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)

add_library(${PROJECT_NAME} SHARED academy.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  ${ONNXRUNTIME_LIB}
)

add_executable(academyCode main.cpp)
ament_target_dependencies(academyCode ${dependencies})
target_link_libraries(academyCode
  ${PROJECT_NAME}
  HAL
  WebGUI
  Frequency
  nlohmann_json::nlohmann_json
  Boost::system
  Boost::thread
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIB}
)

install(TARGETS
  academyCode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()