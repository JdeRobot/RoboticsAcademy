#ifndef THREADUPDATEGUI_H
#define THREADUPDATEGUI_H

#include "../robot/robot.h"
#include "gui.h"

#include <QtGui>

#include <iostream>
#include <sys/time.h>

#define cycle_update_gui 50 //miliseconds

class ThreadUpdateGUI: public QThread
{
public:
    ThreadUpdateGUI(Robot *robot, StateGUI *state);

private:
    GUI* gui;
    Robot* robot;

protected:
    void run();
};

#endif // THREADUPDATEGUI_H
