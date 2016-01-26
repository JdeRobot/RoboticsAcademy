#include "threadupdategui.h"

ThreadUpdateGUI::ThreadUpdateGUI(Robot* robot, StateGUI* state)
{
    this->robot = robot;

    gui = new GUI(robot, state);
    gui->show();

}

void ThreadUpdateGUI::run()
{
    struct timeval a, b;
    long totalb, totala;
    long diff;

    while (true) {
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        gui->updateThreadGUI();

        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;

        if (diff < 0 || diff > cycle_update_gui)
            diff = cycle_update_gui;
        else
            diff = cycle_update_gui - diff;

        /*Sleep Algorithm*/
        usleep(diff * 1000);
        if (diff < 33)
            usleep(33 * 1000);
    }
}
