cmake_minimum_required(VERSION 3.10)
project(MachineVisionLibs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

set(ROS2_PACKAGES
    rclcpp
    rclcpp_action
    ros2srrc_data
    control_msgs
    trajectory_msgs
    builtin_interfaces
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    pcl_filter_msgs
    ament_index_cpp
    cv_bridge
)

foreach(pkg IN LISTS ROS2_PACKAGES)
    find_package(${pkg} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

set(COMMON_INTERFACES_INCLUDE "/home/ws/install/common_interfaces_cpp/include")
set(COMMON_INTERFACES_LIB "/home/ws/install/common_interfaces_cpp/lib/libcommon_interfaces_cpp.so")

set(WEBGUI_ROS_DEPS  rclcpp std_msgs cv_bridge)
set(WEBGUI_SYS_DEPS  Frequency Boost::system nlohmann_json::nlohmann_json)
set(HAL_ROS_DEPS     rclcpp rclcpp_action ros2srrc_data control_msgs trajectory_msgs
                     builtin_interfaces std_msgs sensor_msgs tf2_ros tf2_geometry_msgs
                     pcl_filter_msgs ament_index_cpp)


# --- libFrequency.so ---
add_library(Frequency SHARED src/Frequency.cpp)
target_include_directories(Frequency PUBLIC include)

# --- libWebGUI.so ---
add_library(WebGUI SHARED src/WebGUI.cpp)
target_include_directories(WebGUI PUBLIC include ${COMMON_INTERFACES_INCLUDE} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(WebGUI ${WEBGUI_ROS_DEPS})
target_link_libraries(WebGUI ${WEBGUI_SYS_DEPS} ${COMMON_INTERFACES_LIB} ${OpenCV_LIBS})

# --- libHAL.so ---
add_library(HAL SHARED src/HAL.cpp)
target_include_directories(HAL PUBLIC include ${COMMON_INTERFACES_INCLUDE} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(HAL ${HAL_ROS_DEPS})
target_link_libraries(HAL ${COMMON_INTERFACES_LIB} ${OpenCV_LIBS} yaml-cpp)
