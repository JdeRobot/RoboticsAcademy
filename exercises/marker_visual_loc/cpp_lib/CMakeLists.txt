cmake_minimum_required(VERSION 3.10)
project(VisualMarkerLibs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

set(ROS2_PACKAGES
    rclcpp
    nav_msgs
    gazebo_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    ros_gz_interfaces
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

set(WEBGUI_ROS_DEPS  rclcpp nav_msgs gazebo_msgs tf2 tf2_geometry_msgs sensor_msgs cv_bridge)
set(WEBGUI_SYS_DEPS  Frequency Boost::system nlohmann_json::nlohmann_json ${OpenCV_LIBRARIES} ${COMMON_INTERFACES_LIB})
set(HAL_ROS_DEPS     rclcpp geometry_msgs sensor_msgs std_msgs ros_gz_interfaces nav_msgs cv_bridge)
set(HAL_SYS_DEPS     ${OpenCV_LIBRARIES} ${COMMON_INTERFACES_LIB})

# --- libFrequency.so ---
add_library(Frequency SHARED src/Frequency.cpp)
target_include_directories(Frequency PUBLIC include)

# --- libWebGUI.so ---
add_library(WebGUI SHARED src/WebGUI.cpp src/Map.cpp)
target_include_directories(WebGUI PUBLIC include ${OpenCV_INCLUDE_DIRS} ${COMMON_INTERFACES_INCLUDE})

ament_target_dependencies(WebGUI ${WEBGUI_ROS_DEPS})
target_link_libraries(WebGUI ${WEBGUI_SYS_DEPS})

# --- libHAL.so ---
add_library(HAL SHARED src/HAL.cpp)
target_include_directories(HAL PUBLIC include ${COMMON_INTERFACES_INCLUDE})

ament_target_dependencies(HAL ${HAL_ROS_DEPS})
target_link_libraries(HAL ${HAL_SYS_DEPS})