cmake_minimum_required(VERSION 3.10)
project(VacuumCleanerLibs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

set(ROS2_PACKAGES
    rclcpp
    sensor_msgs
    std_msgs
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

set(WEBGUI_ROS_DEPS  rclcpp sensor_msgs std_msgs cv_bridge)
set(WEBGUI_SYS_DEPS  Frequency Boost::system nlohmann_json::nlohmann_json)


# --- libFrequency.so ---
add_library(Frequency SHARED src/Frequency.cpp)
target_include_directories(Frequency PUBLIC include)

# --- libWebGUI.so ---
# Model.hpp is header-only; no separate library target needed.
add_library(WebGUI SHARED src/WebGUI.cpp)
target_include_directories(WebGUI PUBLIC include ${COMMON_INTERFACES_INCLUDE} ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(WebGUI ${WEBGUI_ROS_DEPS})
target_link_libraries(WebGUI ${WEBGUI_SYS_DEPS} ${COMMON_INTERFACES_LIB} ${OpenCV_LIBS})
