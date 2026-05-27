#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <memory>
#include <thread>
#include <array>

class WebGUINode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

class WebGUI
{
public:
    // Prevent instantiation. WebGUI acts as a global static utility.
    WebGUI() = delete;

    static void show_forces(const std::array<double, 2>& vec1, const std::array<double, 2>& vec2, const std::array<double, 2>& vec3);
    static void show_local_target(const std::array<double, 2>& new_vec);
    static std::array<double, 2> get_next_target();
    static void set_target_x(double x);
    static void set_target_y(double y);
    static void mark_target_reached();

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<WebGUINode> gui_node_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif