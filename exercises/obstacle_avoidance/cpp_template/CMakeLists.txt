cmake_minimum_required(VERSION 3.8)
project(academy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(academyCode
  main.cpp
)
ament_target_dependencies(academyCode ${dependencies})
target_link_libraries(academyCode Boost::system)

install(TARGETS
  academyCode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
