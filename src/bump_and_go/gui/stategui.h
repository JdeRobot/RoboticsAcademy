#ifndef STATEGUI_H
#define STATEGUI_H

#include <QMutex>

#include "../depuratewindow.h"

class StateGUI
{
public:
    StateGUI();

    void setMyAlgorithm();
    bool getMyAlgorithm();

    void setDepurateWindow(DepurateWindow* depurateWindow);
    DepurateWindow* getDepurateWindow();

private:
    QMutex mutex;

    bool myAlgorithm;
    DepurateWindow* depurateWindow;
};

#endif // STATEGUI_H
