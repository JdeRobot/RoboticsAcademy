#include "gui.h"

GUI::GUI(Robot* robot, StateGUI *state)
{

    this->state = state;
    this->robot = robot;

    QGridLayout* mainLayout = new QGridLayout();
    QGridLayout* layoutControl = new QGridLayout();
    QGridLayout* layoutButtons = new QGridLayout();

    camerasWidget = new CamerasWidget(robot);
    glwidget = new GLWidget(state, robot);

    buttonMyAlgorithm = new QPushButton(QIcon(":/images/play.png"), "");
    buttonStopRobot = new QPushButton("Stop Robot");
    checkLaser = new QCheckBox("Laser");
    checkCameras = new QCheckBox("Cameras");
    check3DWorld = new QCheckBox("3DWorld");

    canvasVW = new controlVW();
    laserWidget =new LaserWidget();

    depurateWindow = new DepurateWindow();
    state->setDepurateWindow(depurateWindow);

    int indiceFilaGui = 0;
    layoutControl->addWidget(canvasVW, 0, 0);

    layoutButtons->addWidget(buttonStopRobot, indiceFilaGui, 0);
    layoutButtons->addWidget(buttonMyAlgorithm, indiceFilaGui++, 1);
    layoutButtons->addWidget(check3DWorld, indiceFilaGui++, 1);
    layoutButtons->addWidget(checkCameras, indiceFilaGui++, 1);
    layoutButtons->addWidget(checkLaser, indiceFilaGui++, 1);

    mainLayout->addLayout(layoutControl, 0, 0);
    mainLayout->addLayout(layoutButtons, 0, 1);

    setLayout(mainLayout);

    setVisible(true);

    adjustSize();

    connect(this, SIGNAL(signal_updateGUI()), this, SLOT(on_updateGUI_recieved()));

    connect(buttonStopRobot, SIGNAL(clicked()),this, SLOT(on_buttonStopRobot_clicked()) );
    connect(buttonMyAlgorithm, SIGNAL(clicked()),this, SLOT(on_buttonMyAlgorithm_clicked()) );

    connect(canvasVW, SIGNAL(VW_changed(int,int)), this, SLOT(on_update_canvas_recieved(int, int)));

    connect(check3DWorld, SIGNAL(stateChanged(int)), this, SLOT(on_checks_changed()));
    connect(checkLaser, SIGNAL(stateChanged(int)), this, SLOT(on_checks_changed()));
    connect(checkCameras, SIGNAL(stateChanged(int)), this, SLOT(on_checks_changed()));

    show();

    depurateWindow->setVisible(true);

}

void GUI::on_checks_changed()
{
    glwidget->setVisible(check3DWorld->isChecked());
    camerasWidget->setVisible(checkCameras->isChecked());
    laserWidget->setVisible(checkLaser->isChecked());
}

void GUI::on_update_canvas_recieved(int v, int w)
{
    this->robot->getActuators()->setMotorV((float)v);
    this->robot->getActuators()->setMotorW((float)w);
}

void GUI::on_buttonMyAlgorithm_clicked()
{
    state->setMyAlgorithm();
    if(state->getMyAlgorithm()){
        buttonMyAlgorithm->setIcon(QIcon(":/images/stop.png"));
    }else{
        buttonMyAlgorithm->setIcon(QIcon(":/images/play.png"));
    }
}

void GUI::on_buttonStopRobot_clicked()
{
    canvasVW->Stop();
}

void GUI::updateThreadGUI()
{
    emit signal_updateGUI();
}

void GUI::on_updateGUI_recieved()
{
    camerasWidget->update();
    glwidget->updateGL();
    laserWidget->update(this->robot->getSensors()->getLaserData());
    depurateWindow->update();
}

