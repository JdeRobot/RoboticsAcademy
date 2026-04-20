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
    std::shared_ptr<OdometryNode> get_odometry_node() const;

private:
    std::vector<double> get_pose();

    std::shared_ptr<OdometryNode> odom_node_;
};

#endif