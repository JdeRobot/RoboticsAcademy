#ifndef DEPURATEWINDOW_H
#define DEPURATEWINDOW_H

#include <QtGui>

//Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class DepurateWindow: public QWidget
{
public:
    DepurateWindow();

    void update();

    void setImage(cv::Mat image);

private:
    QMutex mutex;
    QLabel* labelImage;
    cv::Mat image;

};

#endif // DEPURATEWINDOW_H
