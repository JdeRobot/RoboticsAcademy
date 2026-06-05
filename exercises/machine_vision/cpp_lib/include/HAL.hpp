#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <array>
#include <string>
#include <memory>
#include <thread>
#include <opencv2/opencv.hpp>

class HALNode;
class CameraNode;
namespace rclcpp::executors { class MultiThreadedExecutor; }

// Object pose and geometry loaded from the workspace configuration.
struct ObjectInfo {
    std::array<double, 3> position;  // xyz in robot base frame
    double height;
    double width;
    double length;
    std::string shape;
    std::string color;
};

class HAL
{
public:
    // Prevent instantiation. HAL acts as a global static utility.
    HAL() = delete;

    // Absolute joint movement. joints[6] in degrees, speed in [0,1], wait_time in seconds.
    static void MoveAbsJ(const std::array<double, 6>& joints, double speed, double wait_time);

    // Relative single-joint movement. joint_number in [1,6], relative_angle in degrees.
    static void MoveSingleJ(int joint_number, double relative_angle, double speed, double wait_time);

    // Absolute linear (Cartesian) move. xyz in metres, ypr in degrees.
    static void MoveLinear(const std::array<double, 3>& xyz, const std::array<double, 3>& ypr, double speed, double wait_time);

    // Absolute point-to-point (Cartesian) move. xyz in metres, ypr in degrees.
    static void MoveJoint(const std::array<double, 3>& xyz, const std::array<double, 3>& ypr, double speed, double wait_time);

    // Relative linear Cartesian increment. xyz in metres.
    static void MoveRelLinear(const std::array<double, 3>& xyz, double speed, double wait_time);

    // Relative TCP reorientation. ypr in degrees.
    static void MoveRelReor(const std::array<double, 3>& ypr, double speed, double wait_time);

    // Gripper control. relative_closure in [0,100] (0 = fully open, 100 = fully closed).
    static void GripperSet(double relative_closure, double wait_time);

    // TCP pose. Position in metres; orientation as [yaw, pitch, roll] in degrees.
    static std::array<double, 3> get_TCP_position();
    static std::array<double, 3> get_TCP_orientation();

    // Joint states in degrees, ordered shoulder_pan to wrist_3.
    static std::array<double, 6> get_Joint_states();

    // Camera images. camera: "hand" (/hand_camera/image) or "base" (/base_camera/image).
    static cv::Mat getImage(const std::string& camera = "hand");

    // Object and target queries (positions loaded from workspace YAML).
    static std::array<double, 3> get_object_position(const std::string& object_name);
    static ObjectInfo             get_object_info(const std::string& object_name);
    static std::array<double, 3> get_target_position(const std::string& target_name);

    // Move to scan position, trigger environment scan, return to home.
    static void scan_workspace();
    static void buildmap();

    // Point-cloud perception filter control.
    static void start_color_filter(const std::string& color,
                                   int rmax, int rmin,
                                   int gmax, int gmin,
                                   int bmax, int bmin);
    static void stop_color_filter(const std::string& color);
    static void start_shape_filter(const std::string& color, const std::string& shape, double radius);
    static void stop_shape_filter(const std::string& color, const std::string& shape);

    // Convert object diameter to gripper closure percentage. max_open_m is the gripper jaw span at 0%.
    static double gripper_percentage_for(double diameter, double max_open_m = 0.085);

private:
    static void init();
    friend class SystemBootstrapper;

    // Hidden internal state. Not accessible to the user.
    static std::shared_ptr<HALNode>   hal_node_;
    static std::shared_ptr<CameraNode> hand_camera_;
    static std::shared_ptr<CameraNode> base_camera_;
    static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    static std::thread spin_thread_;
};

#endif
