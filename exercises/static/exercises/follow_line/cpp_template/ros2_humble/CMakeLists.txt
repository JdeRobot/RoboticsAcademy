cmake_minimum_required(VERSION 3.8)
project(academy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  std_msgs
)

add_library(${PROJECT_NAME} SHARED
  academy.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(academyCode
  main.cpp
)
ament_target_dependencies(academyCode ${dependencies})
target_link_libraries(academyCode ${PROJECT_NAME})

install(TARGETS
  academyCode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
