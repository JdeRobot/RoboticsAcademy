#include "stategui.h"

StateGUI::StateGUI()
{
    myAlgorithm = false;
}

void StateGUI::setMyAlgorithm()
{
    mutex.lock();
    myAlgorithm = !myAlgorithm;
    mutex.unlock();
}

bool StateGUI::getMyAlgorithm()
{
    mutex.lock();
    bool result = myAlgorithm;
    mutex.unlock();
    return result;
}

void StateGUI::setDepurateWindow(DepurateWindow *depurateWindow)
{
    this->depurateWindow = depurateWindow;
}

DepurateWindow *StateGUI::getDepurateWindow()
{
    return this->depurateWindow;
}



