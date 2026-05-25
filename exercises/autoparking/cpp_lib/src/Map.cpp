#include "Map.hpp"
#include <cmath>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

Map::Map(std::function<LaserData()> laser_f_cb,
         std::function<LaserData()> laser_r_cb,
         std::function<LaserData()> laser_b_cb)
    : laser_f_cb_(laser_f_cb), laser_r_cb_(laser_r_cb), laser_b_cb_(laser_b_cb)
{
}

std::string Map::get_json_data()
{
    LaserValues values = setLaserValues();
    
    json payload;
    
    json lasers_json = json::array();
    for (const auto& laser_array : values.lasers) {
        json single_laser = json::array();
        for (const auto& pt : laser_array) {
            single_laser.push_back({pt.first, pt.second});
        }
        lasers_json.push_back(single_laser);
    }
    
    payload["lasers"] = lasers_json;
    payload["ranges"] = values.ranges;
    
    return payload.dump();
}

cv::Mat Map::RTx(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        1, 0, 0, tx,
        0, std::cos(angle), -std::sin(angle), ty,
        0, std::sin(angle), std::cos(angle), tz,
        0, 0, 0, 1);
}

cv::Mat Map::RTy(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), 0, std::sin(angle), tx,
        0, 1, 0, ty,
        -std::sin(angle), 0, std::cos(angle), tz,
        0, 0, 0, 1);
}

cv::Mat Map::RTz(double angle, double tx, double ty, double tz)
{
    return (cv::Mat_<double>(4, 4) << 
        std::cos(angle), -std::sin(angle), 0, tx,
        std::sin(angle), std::cos(angle), 0, ty,
        0, 0, 1, tz,
        0, 0, 0, 1);
}

Map::LaserValues Map::setLaserValues()
{
    LaserData laser_f = laser_f_cb_();
    LaserData laser_r = laser_r_cb_();
    LaserData laser_b = laser_b_cb_();

    LaserValues result;
    result.lasers.push_back(setSingleLaserValue(laser_f));
    result.lasers.push_back(setSingleLaserValue(laser_r));
    result.lasers.push_back(setSingleLaserValue(laser_b));

    result.ranges.push_back(20.0 * laser_f.maxRange);
    result.ranges.push_back(20.0 * laser_r.maxRange);
    result.ranges.push_back(20.0 * laser_b.maxRange);

    return result;
}

std::vector<std::pair<double, double>> Map::setSingleLaserValue(const LaserData& laser)
{
    std::vector<std::pair<double, double>> laser_return;
    int angle_deg = 180;
    
    if (!laser.values.empty()) {
        angle_deg = static_cast<int>(std::round(laser.maxAngle * 180.0 / CV_PI));
    }

    for (int i = 0; i < angle_deg; ++i) {
        double dist = 0.0;
        
        if (!laser.values.empty() && i < laser.values.size()) {
            dist = 20.0 * laser.values[i];
            if (std::isinf(dist) || std::isnan(dist)) {
                dist = 20.0 * laser.maxRange;
            }
        } else if (!laser.values.empty()) {
            dist = 20.0 * laser.maxRange;
        }

        double angle_rad = i * CV_PI / 180.0;
        laser_return.push_back({dist, angle_rad});
    }

    return laser_return;
}