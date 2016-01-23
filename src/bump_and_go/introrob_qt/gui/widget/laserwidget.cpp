#include "laserwidget.h"

LaserWidget::LaserWidget()
{

    setAutoFillBackground(true);

    QPalette Pal(palette());
    // set black background
    Pal.setColor(QPalette::Background, Qt::black);
    setAutoFillBackground(true);
    setPalette(Pal);

    setMaximumSize(530, 250);

    setWindowTitle("Laser");
}

void LaserWidget::update(std::vector<float> laserData)
{

    mutex.lock();

    this->laserData = laserData;

    mutex.unlock();

}

void LaserWidget::paintEvent(QPaintEvent *)
{
    int _width = width();
    int _height = height();


    int width = 2;
    QPen pen;

    QPainter painter(this);
    painter.setPen(pen);

    pen = QPen(Qt::blue, width);
    painter.setPen(pen);

    // Centro del widget y abajo del todo
    painter.translate(QPoint(80, 50));

    float PI = 3.1416;

    for (int i = 0; i < this->laserData.size(); i++) {
        painter.drawLine(QPointF(180 + ((this->laserData[i] / 45)*(cos((i) * PI / 180))),
                                 180 - ((this->laserData[i] / 45)*(sin((i) * PI / 180)))),
                         QPointF(180 + ((this->laserData[i + 1] / 45)*(cos((i + 1) * PI / 180))),
                                 180 - ((this->laserData[i + 1] / 45)*(sin((i + 1) * PI / 180)))));
    }
}


