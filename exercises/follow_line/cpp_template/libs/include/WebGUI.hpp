#ifndef INCLUDE_WEBGUI_HPP_
#define INCLUDE_WEBGUI_HPP_

#include <opencv2/opencv.hpp>

class WebGUI
{
public:
    static void show_image(const cv::Mat& image);

private:
    static void init();
    friend class SystemBootstrapper;
};

#endif