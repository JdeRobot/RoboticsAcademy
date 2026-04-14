cmake_minimum_required(VERSION 3.10)
project(GlobalNavigationLibs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

set(ROS2_PACKAGES
    rclcpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
)

foreach(pkg IN LISTS ROS2_PACKAGES)
    find_package(${pkg} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)

set(COMMON_INTERFACES_INCLUDE "/home/ws/install/common_interfaces_cpp/include")
set(COMMON_INTERFACES_LIB "/home/ws/install/common_interfaces_cpp/lib/libcommon_interfaces_cpp.so")

set(HAL_ROS_DEPS     rclcpp geometry_msgs nav_msgs)
set(WEBGUI_ROS_DEPS  rclcpp nav_msgs geometry_msgs sensor_msgs cv_bridge)
set(WEBGUI_SYS_DEPS  Frequency Boost::system nlohmann_json::nlohmann_json ${OpenCV_LIBS})


# --- libFrequency.so ---
add_library(Frequency SHARED src/Frequency.cpp)
target_include_directories(Frequency PUBLIC include)

# --- libHAL.so ---
add_library(HAL SHARED src/HAL.cpp)
target_include_directories(HAL PUBLIC include ${COMMON_INTERFACES_INCLUDE})

ament_target_dependencies(HAL ${HAL_ROS_DEPS})
target_link_libraries(HAL ${COMMON_INTERFACES_LIB})

# --- libWebGUI.so ---
add_library(WebGUI SHARED src/WebGUI.cpp src/Map.cpp)
target_include_directories(WebGUI PUBLIC 
    include 
    ${COMMON_INTERFACES_INCLUDE} 
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(WebGUI ${WEBGUI_ROS_DEPS})
target_link_libraries(WebGUI ${WEBGUI_SYS_DEPS} ${COMMON_INTERFACES_LIB})