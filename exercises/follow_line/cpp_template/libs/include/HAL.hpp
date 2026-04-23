#ifndef INCLUDE_HAL_HPP_
#define INCLUDE_HAL_HPP_

#include <opencv2/opencv.hpp>

class HAL
{
public:
    static void set_v(const float velocity);
    static void set_w(const float velocity);
    static cv::Mat get_image();

private:
    static void init();
    friend class SystemBootstrapper;
};

#endif