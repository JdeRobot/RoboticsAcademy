#include "robot.h"

Robot::Robot(Ice::CommunicatorPtr ic)
{
    sensors = new Sensors(ic);
    actuators = new Actuators(ic);

    pthread_mutex_init (&mutex, NULL);

}

Sensors* Robot::getSensors()
{
    return this->sensors;
}

Actuators* Robot::getActuators()
{
    return this->actuators;
}

void Robot::update()
{
    this->sensors->update();
    this->actuators->update();
}




