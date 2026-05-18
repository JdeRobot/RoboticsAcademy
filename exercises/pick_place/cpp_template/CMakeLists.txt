cmake_minimum_required(VERSION 3.8)
project(academy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_interfaces_cpp REQUIRED)

find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

set(dependencies
  rclcpp
  sensor_msgs
  std_msgs
  common_interfaces_cpp
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/include
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)

add_library(${PROJECT_NAME} SHARED academy.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
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
)

install(TARGETS
  academyCode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
