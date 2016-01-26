#include "cameraswidget.h"

CamerasWidget::CamerasWidget(Robot* robot)
{

    this->robot = robot;

    QGridLayout* mainLayout = new QGridLayout();

    labelImage1 = new QLabel();
    labelImage2 = new QLabel();

    int  indiceFilaGui = 0;
    mainLayout->addWidget(labelImage1, indiceFilaGui, 0);
    mainLayout->addWidget(labelImage2, indiceFilaGui++, 1);

    setLayout(mainLayout);

    setWindowTitle("Cameras");

}

void CamerasWidget::update()
{
    cv::Mat frame1 = this->robot->getSensors()->getCamera1();
    cv::Mat frame2 = this->robot->getSensors()->getCamera2();

    cv::resize(frame1, frame1, cv::Size(320, 240));
    cv::resize(frame2, frame2, cv::Size(320, 240));

    QImage imageQt1 = QImage((const unsigned char*)(frame1.data),
                            frame1.cols,
                            frame1.rows,
                            frame1.step,
                            QImage::Format_RGB888);

    QImage imageQt2 = QImage((const unsigned char*)(frame2.data),
                            frame2.cols,
                            frame2.rows,
                            frame2.step,
                            QImage::Format_RGB888);

    labelImage1->setPixmap(QPixmap::fromImage(imageQt1));
    labelImage2->setPixmap(QPixmap::fromImage(imageQt2));
}

void CamerasWidget::mousePressEvent(QMouseEvent* event)
{
    float x = event->x();
    float y = event->y();

    QPoint p = labelImage1->mapFromParent(event->pos());
    if(p.x()>320){
        p = labelImage2->mapFromParent(event->pos());
    }

    std::cout << " --------------------" << std::endl;
    std::cout << " X: " << p.x() << " Y: " << p.y() << std::endl;
    std::cout << " X: " << x << " Y: " << y << std::endl;

}


