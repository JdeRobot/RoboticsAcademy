#include "WebGUI.hpp"

WebGUI::WebGUI()
    : BaseWebGUI("webgui_node", "127.0.0.1", "2303", 20.0)
{
    odom_node_ = std::make_shared<OdometryNode>("/odom", "webgui_odom_node");
}

std::vector<rclcpp::Node::SharedPtr> WebGUI::get_nodes()
{
    auto nodes = BaseWebGUI::get_nodes();
    nodes.push_back(odom_node_);
    return nodes;
}

std::vector<double> WebGUI::get_pose()
{
    if (!odom_node_) {
        return {201.0, 85.5, 0.0};
    }

    Pose3d pose = odom_node_->getPose3d();

    if (pose.x == 0.0 && pose.y == 0.0 && pose.yaw == 0.0) {
        return {201.0, 85.5, 0.0};
    }

    const double map_x = -30.0 * pose.x + 171.0;
    const double map_y = 15.0 * pose.y + 63.0;

    return {map_x, map_y, pose.yaw};
}

json WebGUI::update_gui()
{
    auto pose_data = get_pose();
    json map_j = {pose_data.at(0), pose_data.at(1), pose_data.at(2)};

    json payload;
    payload["map"] = map_j.dump();

    return payload;
}