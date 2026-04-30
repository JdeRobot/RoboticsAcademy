#include "WebGUI.hpp"

std::shared_ptr<WebGUI> gui = nullptr;

static const std::string base64_chars = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
    std::string ret;
    int i = 0, j = 0;
    unsigned char char_array_3[3], char_array_4[4];

    while (in_len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for(i = 0; (i < 4); i++) ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for(j = i; j < 3; j++) char_array_3[j] = '\0';
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++) ret += base64_chars[char_array_4[j]];
        while((i++ < 3)) ret += '=';
    }
    return ret;
}

WebGUI::WebGUI(const std::string& host, const std::string& port, double freq)
    : MeasuringThreadingGUI(host, port, freq), has_predict_pose_(false) {
    
    setup_ros2();
    
    map_ = std::make_shared<Map>(
        std::bind(&WebGUI::get_pose3d, this),
        std::bind(&WebGUI::get_odom, this)
    );

    MeasuringThreadingGUI::start();
}

WebGUI::~WebGUI() {
    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
}

void WebGUI::setup_ros2() {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    bridge_node_ = rclcpp::Node::make_shared("gui_bridge_node_filter");
    
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.transient_local();

    estimated_pose_sub_ = bridge_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/webgui/estimated_pose", 
        qos_profile, 
        std::bind(&WebGUI::estimated_pose_callback, this, std::placeholders::_1)
    );

    real_odom_node_ = std::make_shared<OdometryNode>("/turtlebot3/odom", "real_odom_node");
    noisy_odom_node_ = std::make_shared<OdometryNode>("/turtlebot3/odom_noisy", "noisy_odom_node");
    camera_node_ = std::make_shared<CameraNode>("/webgui/image_debug");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(bridge_node_);
    executor_->add_node(real_odom_node_);
    executor_->add_node(noisy_odom_node_);
    executor_->add_node(camera_node_);

    executor_thread_ = std::thread([this]() {
        executor_->spin();
    });
}

void WebGUI::estimated_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double yaw = quat2Yaw(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    WebGUI::showEstimatedPose(std::make_tuple(x, y, yaw));
}

Pose3d WebGUI::get_pose3d() {
    return real_odom_node_->getPose3d();
}

Pose3d WebGUI::get_odom() {
    return noisy_odom_node_->getPose3d();
}

void WebGUI::setImage(const cv::Mat& image) {
    std::lock_guard<std::mutex> lock(image_lock_);
    image.copyTo(image_);
}

void WebGUI::setEstimatedRobotPose(const std::tuple<double, double, double>& pose) {
    predict_pose_ = pose;
    has_predict_pose_ = true;
}

void WebGUI::showImage(const cv::Mat& image) {
    if (gui) {
        gui->setImage(image);
    }
}

void WebGUI::showEstimatedPose(const std::tuple<double, double, double>& pose) {
    if (gui) {
        double x = std::get<0>(pose);
        double y = std::get<1>(pose);
        double yaw = std::get<2>(pose);

        double scale_y = -85.0;
        double offset_y = -6.88;
        y = scale_y * (offset_y - y);

        double scale_x = 83.0;
        double offset_x = 8.0;
        x = scale_x * (offset_x - x);

        gui->setEstimatedRobotPose(std::make_tuple(x, y, yaw));
    }
}

void WebGUI::update_gui() {
    nlohmann::json payload;

    auto real_pose_vec = map_->getRobotCoordinates();
    if (!real_pose_vec.empty()) {
        std::string pose_str = "(";
        for (size_t i = 0; i < real_pose_vec.size(); ++i) {
            pose_str += std::to_string(real_pose_vec[i]);
            if (i < real_pose_vec.size() - 1) pose_str += ", ";
        }
        pose_str += ")";
        payload["real_pose"] = pose_str;
    }

    auto noisy_pose_vec = map_->getRobotCoordinatesWithNoise();
    if (!noisy_pose_vec.empty()) {
        std::string noisy_str = "(";
        for (size_t i = 0; i < noisy_pose_vec.size(); ++i) {
            noisy_str += std::to_string(noisy_pose_vec[i]);
            if (i < noisy_pose_vec.size() - 1) noisy_str += ", ";
        }
        noisy_str += ")";
        payload["noisy_pose"] = noisy_str;
    }

    if (has_predict_pose_) {
        payload["estimate_pose"] = "(" + std::to_string(std::get<0>(predict_pose_)) + ", " +
                                   std::to_string(std::get<1>(predict_pose_)) + ", " +
                                   std::to_string(std::get<2>(predict_pose_)) + ")";
    }

    auto cam_img = camera_node_->getImage();
    if (cam_img && !cam_img->data.empty()) {
        setImage(cam_img->data);
    }

    cv::Mat current_img;
    {
        std::lock_guard<std::mutex> lock(image_lock_);
        if (!image_.empty()) image_.copyTo(current_img);
    }

    nlohmann::json payload_img;
    if (!current_img.empty()) {
        std::vector<uchar> buf;
        cv::imencode(".JPEG", current_img, buf);
        payload_img["image"] = base64_encode(buf.data(), buf.size());
        payload_img["shape"] = {current_img.rows, current_img.cols, current_img.channels()};
    } else {
        payload_img["image"] = nullptr;
        payload_img["shape"] = 0;
    }
    
    payload["image"] = payload_img.dump();
    send_to_client(payload.dump());
}