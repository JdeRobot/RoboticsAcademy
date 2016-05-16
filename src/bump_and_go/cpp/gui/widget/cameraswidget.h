#ifndef CAMERASWIDGET_H
#define CAMERASWIDGET_H

#include <QtGui>

#include "../../robot/robot.h"

class CamerasWidget: public QWidget
{
    Q_OBJECT

public:
    CamerasWidget(Robot *robot);

    void update();

private:
    Robot* robot;
    QLabel* labelImage1;
    QLabel* labelImage2;

protected:
    void mousePressEvent(QMouseEvent* event);
public slots:



};

#endif // CAMERASWIDGET_H
