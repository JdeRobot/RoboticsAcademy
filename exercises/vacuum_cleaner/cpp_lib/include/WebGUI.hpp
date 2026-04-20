#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include "common_interfaces_cpp/webgui/WebGUIBridge.hpp"
#include "common_interfaces_cpp/hal/odometry.hpp"
#include <memory>
#include <vector>

class WebGUI : public BaseWebGUI
{
public:
    WebGUI();
    ~WebGUI() override = default;

    json update_gui() override;
    std::vector<rclcpp::Node::SharedPtr> get_nodes();

private:
    std::vector<double> get_pose();

    std::shared_ptr<OdometryNode> odom_node_;
};

#endif // INCLUDE_WEBGUI_HPP_