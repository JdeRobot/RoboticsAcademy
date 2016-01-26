#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <QMutex>

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <jderobot/motors.h>

class Actuators
{
public:
    Actuators(Ice::CommunicatorPtr ic);
    void update();
    void setActuators();

    // GETS
    float getMotorV();
    float getMotorW();
    float getMotorL();

    //SETS
    void setMotorV(float motorV);
    void setMotorW(float motorW);
    void setMotorL(float motorL);
    void setMotorSTOP();

private:

    QMutex mutex;

    Ice::CommunicatorPtr ic;

    // ICE INTERFACES
    jderobot::MotorsPrx mprx;
 
    //ICE interfaces available for connection on demand
    bool motorsON ;

    float motorVin;
    float motorWin;
    float motorLin;

    float motorVout;
    float motorWout;
    float motorLout;

};
#endif // ACTUATORS_H
