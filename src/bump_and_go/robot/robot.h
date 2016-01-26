#ifndef ROBOT_H
#define ROBOT_H

#include <QtGui>

#include "sensors.h"
#include "actuators.h"

#include <stdio.h>

class Robot: public QObject
{
    Q_OBJECT

public:
    Robot(Ice::CommunicatorPtr ic);

    void update();

    Actuators* getActuators();
    Sensors* getSensors();

private:
    pthread_mutex_t mutex;

    Sensors* sensors;
    Actuators* actuators;
};

#endif // ROBOT_H
