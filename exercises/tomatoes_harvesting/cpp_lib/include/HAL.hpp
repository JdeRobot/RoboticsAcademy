#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

// HAL of the MMO-500 harvester in the tomato greenhouse
// Arm goals are relative to the base so they stay valid while the robot drives
class HAL
{
public:
  // Prevent instantiation. HAL acts as a global static utility.
  HAL() = delete;

  // Base pose in the world frame with distances in meters and angles in radians
  struct Pose3d
  {
    double x, y, z, h, yaw, pitch, roll;
    // Quaternion [w, x, y, z]
    std::array<double, 4> q;
    double timeStamp;
  };

  struct LaserData
  {
    // Distances in meters
    std::vector<float> values;
    double minAngle, maxAngle, minRange, maxRange, timeStamp;
  };

  // Grasp point relative to the base and the direction its fingers point to
  struct ArmPose
  {
    double x, y, z, heading;
  };

  /// BASE ///

  static Pose3d get_pose3d();

  static LaserData get_front_laser_data();

  static LaserData get_back_laser_data();

  // Forward speed of the mecanum base in m/s, positive forwards
  static void set_vx(double velocity);

  // Sideways speed of the mecanum base in m/s, positive to the left
  static void set_vy(double velocity);

  // Yaw rate in rad/s
  static void set_w(double velocity);

  /// ARM ///

  // Moves the grasp point between the fingers to x, y, z in m relative to the base
  // The head stays level and its fingers point towards heading, an angle around the
  // vertical measured like the base yaw. The default points to the right of the robot
  // Returns true once the arm is there
  static bool move_arm(double x, double y, double z, double heading = -M_PI / 2);

  // Current grasp point, nothing until the arm reports its first pose
  static std::optional<ArmPose> get_arm_pose();

  // Folds the arm back to the pose the robot starts in
  static bool home_arm();

  /// HARVEST HEAD ///

  // Closes the fingers until they squeeze with force in N and returns the force reached
  // Returns 0.0 when the fingers closed completely without touching anything
  static double grip(double force);

  // Opens the fingers, the shaft winds back to zero on its own meanwhile
  static bool release();

  // Spins the fingers this many turns, negative turns spin the other way
  static bool twist(double turns);

  /// WRIST CAMERA ///

  // BGR image of the wrist camera
  static cv::Mat get_image();

  // Depth of every pixel in m as CV_32F, inf where nothing is in range
  static cv::Mat get_depth_image();

  // Point seen at pixel column u and row v as [x, y, z] in m relative to the base
  // Returns nothing when that pixel has no depth or the arm pose is still unknown
  static std::optional<std::array<double, 3>> pixel_to_point(double u, double v);
};

#endif
