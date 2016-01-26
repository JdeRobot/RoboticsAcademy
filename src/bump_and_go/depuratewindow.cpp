#include "depuratewindow.h"

DepurateWindow::DepurateWindow()
{
    QGridLayout* mainLayout = new QGridLayout();

    labelImage = new QLabel();

    mainLayout->addWidget(labelImage, 0, 0);

    setLayout(mainLayout);

}

void DepurateWindow::update()
{
    mutex.lock();
    if(image.data){

        QImage imageQt = QImage((const unsigned char*)(image.data),
                                image.cols,
                                image.rows,
                                image.step,
                                QImage::Format_RGB888);

        labelImage->setPixmap(QPixmap::fromImage(imageQt));
    }
    mutex.unlock();

}

void DepurateWindow::setImage(cv::Mat image)
{
    mutex.lock();
    this->image = image.clone();
    mutex.unlock();

}

