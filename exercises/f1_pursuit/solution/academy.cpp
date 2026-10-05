#include "HAL.hpp"
#include "Frequency.hpp"

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

const std::vector<double> ROWS = {0.88, 0.80, 0.72, 0.63, 0.54};
constexpr int MIN_RUN = 3;

constexpr double KP = 1.35;
constexpr double KD = 0.055;
constexpr double K_CURVE = 0.9;
constexpr double W_CLAMP = 1.6;

constexpr double V_STRAIGHT = 4.2;
constexpr double V_CORNER = 1.3;
constexpr double CORNER_BRAKE = 1.5;

cv::Mat red_mask(const cv::Mat &image) {
  cv::Mat hsv, lo, hi;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  // Red wraps the hue origin, so it takes two ranges
  cv::inRange(hsv, cv::Scalar(0, 110, 70), cv::Scalar(10, 255, 255), lo);
  cv::inRange(hsv, cv::Scalar(170, 110, 70), cv::Scalar(180, 255, 255), hi);
  return lo | hi;
}

std::vector<double> row_runs(const cv::Mat &mask, int y) {
  std::vector<double> mids;
  const uchar *row = mask.ptr<uchar>(y);
  int start = -1;
  for (int x = 0; x <= mask.cols; ++x) {
    const bool on = (x < mask.cols) && row[x] > 0;
    if (on && start < 0) {
      start = x;
    } else if (!on && start >= 0) {
      if (x - start >= MIN_RUN)
        mids.push_back((start + x - 1) / 2.0);
      start = -1;
    }
  }
  return mids;
}

// Sample the line bottom to top, each row anchored on the one below it. Both
// lanes are the same red, so continuity is what separates ours from the rival's.
std::vector<double> follow_line(const cv::Mat &mask, double seed, bool *found) {
  const int h = mask.rows, w = mask.cols;
  std::vector<double> columns;
  double anchor = seed;
  bool first = true;
  for (double frac : ROWS) {
    const std::vector<double> mids = row_runs(mask, int(h * frac));
    if (mids.empty()) {
      columns.push_back(-1.0);
      continue;
    }
    double best = mids[0];
    for (double m : mids)
      if (std::fabs(m - anchor) < std::fabs(best - anchor))
        best = m;
    if (!first && std::fabs(best - anchor) > w * 0.45) {
      columns.push_back(-1.0);
      continue;
    }
    anchor = best;
    first = false;
    columns.push_back(best);
  }
  *found = columns[0] >= 0.0;
  return columns;
}

}  // namespace

void exercise() {
  Frequency frequency;
  double track_x = -1.0;
  double last_error = 0.0;

  while (true) {
    cv::Mat image = HAL::get_image();
    if (image.empty()) {
      frequency.tick(50);
      continue;
    }

    const int w = image.cols;
    const cv::Mat mask = red_mask(image);
    if (track_x < 0.0)
      track_x = w / 2.0;

    bool have_near = false;
    const std::vector<double> columns = follow_line(mask, track_x, &have_near);

    if (!have_near) {
      HAL::set_v(0.7f);
      HAL::set_w(track_x < w / 2.0 ? 0.7f : -0.7f);
    } else {
      const double near = columns[0];
      track_x = near;
      const double error = (near - w / 2.0) / (w / 2.0);

      double far = near;
      for (auto it = columns.rbegin(); it != columns.rend(); ++it) {
        if (*it >= 0.0) {
          far = *it;
          break;
        }
      }
      const double curve = (far - near) / (w / 2.0);

      double steer = -(KP * error + KD * (error - last_error) + K_CURVE * curve);
      steer = std::max(-W_CLAMP, std::min(W_CLAMP, steer));
      last_error = error;

      const double bend =
          std::min(1.0, std::fabs(curve) * CORNER_BRAKE + 0.4 * std::fabs(error));
      const double speed = V_STRAIGHT - (V_STRAIGHT - V_CORNER) * bend;

      HAL::set_v(static_cast<float>(std::max(V_CORNER, speed)));
      HAL::set_w(static_cast<float>(steer));
    }

    if (HAL::is_caught()) {
      std::cout << "caught the rival" << std::endl;
    }

    frequency.tick(50);
  }
}
