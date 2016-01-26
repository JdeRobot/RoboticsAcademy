#ifndef GUI_H
#define GUI_H

#include <QtGui>

#include "../robot/robot.h"
#include "stategui.h"

#include "widget/controlvw.h"
#include "widget/cameraswidget.h"
#include "widget/glwidget.h"
#include "widget/laserwidget.h"

#include "../depuratewindow.h"

class GUI:public QWidget
{
    Q_OBJECT

public:
    GUI(Robot* robot, StateGUI* state);
    void updateThreadGUI();

private:
    QPushButton* buttonMyAlgorithm;
    QPushButton* buttonStopRobot;

    controlVW* canvasVW;
    CamerasWidget* camerasWidget;
    GLWidget* glwidget;
    LaserWidget* laserWidget;
    DepurateWindow* depurateWindow;

    Robot* robot;
    StateGUI* state;

    QCheckBox* check3DWorld;
    QCheckBox* checkCameras;
    QCheckBox* checkLaser;


signals:
    void signal_updateGUI();

public slots:
    void on_updateGUI_recieved();
    void on_buttonMyAlgorithm_clicked();
    void on_buttonStopRobot_clicked();

    void on_update_canvas_recieved(int v, int w);
    void on_checks_changed();

};

#endif // GUI_H
