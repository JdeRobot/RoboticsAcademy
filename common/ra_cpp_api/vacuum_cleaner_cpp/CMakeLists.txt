cmake_minimum_required(VERSION 3.10)
project(VacuumCleanerLibs CXX)

# 1. Standard C++ Configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 2. Find core dependencies
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)

# 3. Exact paths extracted from the Docker container
# Hardcoding the paths to the common_interfaces_cpp installation
set(COMMON_INTERFACES_INCLUDE "/home/ws/install/common_interfaces_cpp/include")
set(COMMON_INTERFACES_LIB "/home/ws/install/common_interfaces_cpp/lib/libcommon_interfaces_cpp.so")

# TARGET 1: libFrequency.so
add_library(Frequency SHARED src/Frequency.cpp)
target_include_directories(Frequency PUBLIC include)

# TARGET 2: libWebGUI.so
add_library(WebGUI SHARED src/WebGUI.cpp)
target_include_directories(WebGUI PUBLIC include)

# Link WebGUI against its specific dependencies
target_link_libraries(WebGUI
    PUBLIC
    Frequency 
    rclcpp::rclcpp
    nav_msgs::nav_msgs__rosidl_generator_c
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    gazebo_msgs::gazebo_msgs__rosidl_generator_c
    gazebo_msgs::gazebo_msgs__rosidl_typesupport_cpp
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
    Boost::system
    nlohmann_json::nlohmann_json
)

# TARGET 3: libHAL.so
add_library(HAL SHARED src/HAL.cpp)

# Include both local headers and the absolute path to the common interfaces
target_include_directories(HAL PUBLIC 
    include
    ${COMMON_INTERFACES_INCLUDE}
)

# Link HAL against ROS 2 and the absolute path to the precompiled interfaces .so
target_link_libraries(HAL
    PUBLIC
    rclcpp::rclcpp
    ${COMMON_INTERFACES_LIB}
)