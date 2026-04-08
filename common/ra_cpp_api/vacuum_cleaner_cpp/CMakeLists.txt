cmake_minimum_required(VERSION 3.10)
project(VacuumCleanerLibs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)

set(COMMON_INTERFACES_INCLUDE "/home/ws/install/common_interfaces_cpp/include")
set(COMMON_INTERFACES_LIB "/home/ws/install/common_interfaces_cpp/lib/libcommon_interfaces_cpp.so")

# ==========================================
# TARGET 1: libFrequency.so
# ==========================================
add_library(Frequency SHARED src/Frequency.cpp)
target_include_directories(Frequency PUBLIC include)

# ==========================================
# TARGET 2: libWebGUI.so
# ==========================================
add_library(WebGUI SHARED src/WebGUI.cpp)
target_include_directories(WebGUI PUBLIC include)

ament_target_dependencies(WebGUI rclcpp nav_msgs gazebo_msgs tf2 tf2_geometry_msgs)
target_link_libraries(WebGUI PUBLIC Frequency Boost::system nlohmann_json::nlohmann_json)

# ==========================================
# TARGET 3: libHAL.so
# ==========================================
add_library(HAL SHARED src/HAL.cpp)
target_include_directories(HAL PUBLIC include ${COMMON_INTERFACES_INCLUDE})

ament_target_dependencies(HAL rclcpp geometry_msgs sensor_msgs std_msgs)
target_link_libraries(HAL PUBLIC ${COMMON_INTERFACES_LIB})