#include "HAL.hpp"
#include "common_interfaces_cpp/hal/camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

// CameraParameters
//
// Holds intrinsic (K) and extrinsic (RT) matrices for one camera and
// implements the geometric operations exposed through HAL.
//
// Values are loaded from 3d_reconstruction_conf.yml; for the simulation
// both cameras share the same K and image size.

namespace {

struct CameraParameters {
  cv::Matx33d K;  // 3 × 3 intrinsic matrix
  cv::Matx44d RT; // 4 × 4 extrinsic matrix  [R | t]
                  //                          [0   1]
  int width;
  int height;

  // Backproject a 2D homogeneous point [x, y, h] into a 3D ray [X, Y, Z, 1].
  // Faithful translation of ListenerParameters.backproject() from
  // parameters_camera.py.
  cv::Vec4d backproject(const cv::Vec3d &p) const {
    double myin_h = K(0, 0);
    double myin_x = p[0] * K(0, 0) / p[2];
    double myin_y = p[1] * K(0, 0) / p[2];

    // Inverse intrinsic (camera matrix is upper-triangular, closed-form
    // inverse)
    double ik11 = 1.0 / K(0, 0);
    double ik13 = -K(0, 2) / K(0, 0);
    double ik22 = 1.0 / K(1, 1);
    double ik23 = -K(1, 2) / K(1, 1);
    double ik33 = 1.0 / K(2, 2);

    double a1 = ik11 * myin_x + ik13 * myin_h;
    double a2 = ik22 * myin_y + ik23 * myin_h;
    double a3 = ik33 * myin_h;

    // Inverse rotation  R^{-1} = R^T
    double b1 = RT(0, 0) * a1 + RT(1, 0) * a2 + RT(2, 0) * a3;
    double b2 = RT(0, 1) * a1 + RT(1, 1) * a2 + RT(2, 1) * a3;
    double b3 = RT(0, 2) * a1 + RT(1, 2) * a2 + RT(2, 2) * a3;

    // Apply translation offset (only RT[0,3] is non-zero for these cameras)
    return cv::Vec4d(b1 + RT(0, 3), b2, b3, 1.0);
  }

  // Project a 3D homogeneous point onto the image plane.
  // Sets point3d[3] = -1 before computing, matching Python semantics.
  cv::Vec3d project(cv::Vec4d &point3d) const {
    point3d[3] = -1.0;

    double a1 = RT(0, 0) * point3d[0] + RT(0, 1) * point3d[1] +
                RT(0, 2) * point3d[2] + RT(0, 3) * point3d[3];
    double a2 = RT(1, 0) * point3d[0] + RT(1, 1) * point3d[1] +
                RT(1, 2) * point3d[2] + RT(1, 3) * point3d[3];
    double a3 = RT(2, 0) * point3d[0] + RT(2, 1) * point3d[1] +
                RT(2, 2) * point3d[2] + RT(2, 3) * point3d[3];

    double out_x = K(0, 0) * a1 + K(0, 1) * a2 + K(0, 2) * a3;
    double out_y = K(1, 0) * a1 + K(1, 1) * a2 + K(1, 2) * a3;
    double out_h = K(2, 0) * a1 + K(2, 1) * a2 + K(2, 2) * a3;

    if (out_h != 0.0) {
      out_x /= out_h;
      out_y /= out_h;
      out_h = 1.0;
    }
    return cv::Vec3d(out_x, out_y, out_h);
  }

  // Flip y-axis: graphic (top-left origin) ↔ optical (bottom-left origin).
  cv::Vec3d grafic_to_optical(const cv::Vec3d &p) const {
    return cv::Vec3d(p[0], static_cast<double>(height) - 1.0 - p[1], p[2]);
  }

  cv::Vec3d optical_to_grafic(const cv::Vec3d &p) const {
    return cv::Vec3d(p[0], static_cast<double>(height) - 1.0 - p[1], p[2]);
  }

  cv::Vec3d get_camera_position() const {
    return cv::Vec3d(RT(0, 3), RT(1, 3), RT(2, 3));
  }
};

// Hardcoded calibration data from 3d_reconstruction_conf.yml.
//
// K list layout:  [K00, K01, K02, -, K10, K11, K12, -, K20, K21, K22, -]
// K = [[240, 0, 320], [0, 240, 240], [0, 0, 1]]
//
// RT list layout: row-major 4×4 (last row always [0,0,0,1])

const CameraParameters kCamLeft = {
    cv::Matx33d(240, 0, 320, 0, 240, 240, 0, 0, 1),
    cv::Matx44d(1, 0, 0, -110, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1), 640, 480};

const CameraParameters kCamRight = {
    cv::Matx33d(240, 0, 320, 0, 240, 240, 0, 0, 1),
    cv::Matx44d(1, 0, 0, 110, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1), 640, 480};

const CameraParameters &select(const std::string &lr) {
  if (lr == "left")
    return kCamLeft;
  if (lr == "right")
    return kCamRight;
  throw std::invalid_argument(
      "HAL: camera selector must be \"left\" or \"right\"");
}

} // namespace

// Static member definitions
std::shared_ptr<CameraNode> HAL::camera_left_ = nullptr;
std::shared_ptr<CameraNode> HAL::camera_right_ = nullptr;
std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> HAL::executor_ =
    nullptr;
std::thread HAL::spin_thread_;

void HAL::init() {
  if (camera_left_)
    return;

  camera_left_ = std::make_shared<CameraNode>(
      "/turtlebot2/camera_left/image_raw", "hal_camera_left");
  camera_right_ = std::make_shared<CameraNode>(
      "/turtlebot2/camera_right/image_raw", "hal_camera_right");

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(camera_left_);
  executor_->add_node(camera_right_);

  spin_thread_ = std::thread([]() { executor_->spin(); });
  spin_thread_.detach();
}

// Camera image access (blocks until first frame is available)
cv::Mat HAL::get_image_left() {
  if (!camera_left_)
    return cv::Mat();
  auto image = camera_left_->getImage();
  while (!image && rclcpp::ok()) {
    std::this_thread::sleep_for(5ms);
    image = camera_left_->getImage();
  }
  return image ? image->data.clone() : cv::Mat();
}

cv::Mat HAL::get_image_right() {
  if (!camera_right_)
    return cv::Mat();
  auto image = camera_right_->getImage();
  while (!image && rclcpp::ok()) {
    std::this_thread::sleep_for(5ms);
    image = camera_right_->getImage();
  }
  return image ? image->data.clone() : cv::Mat();
}

// Geometric operations, delegate to the chosen CameraParameters
cv::Vec3d HAL::grafic_to_optical(const std::string &lr,
                                 const cv::Vec3d &point2d) {
  return select(lr).grafic_to_optical(point2d);
}

cv::Vec3d HAL::optical_to_grafic(const std::string &lr,
                                 const cv::Vec3d &point2d) {
  return select(lr).optical_to_grafic(point2d);
}

cv::Vec4d HAL::backproject(const std::string &lr, const cv::Vec3d &point2d) {
  return select(lr).backproject(point2d);
}

cv::Vec3d HAL::project(const std::string &lr, cv::Vec4d &point3d) {
  return select(lr).project(point3d);
}

cv::Vec3d HAL::get_camera_position(const std::string &lr) {
  return select(lr).get_camera_position();
}

// Scale a reconstructed 3D point for the frontend 3D viewer.
cv::Vec3d HAL::project_3d_scene(const cv::Vec4d &point3d) {
  return cv::Vec3d(point3d[0] / 100.0, point3d[1] / 100.0 + 12.0,
                   point3d[2] / 100.0);
}
