#include "threadupdaterobot.h"

ThreadUpdateRobot::ThreadUpdateRobot(Robot *robot, StateGUI *state)
{
    this->robot = robot;
    this->state = state;
}

void ThreadUpdateRobot::run()
{
    struct timeval a, b;
    long totalb, totala;
    long diff;

    while (true) {
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        this->robot->update();
        this->robot->getActuators()->setActuators();


        if(state->getMyAlgorithm()){
            RunNavigationAlgorithm();
        }

        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;

        if (diff < 0 || diff > cycle_update_robot)
            diff = cycle_update_robot;
        else
            diff = cycle_update_robot - diff;


        /*Sleep Algorithm*/
        usleep(diff * 1000);
        if (diff < 33)
            usleep(33 * 1000);
    }
}
