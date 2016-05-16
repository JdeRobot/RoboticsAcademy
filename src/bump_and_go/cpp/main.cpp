#include "robot/robot.h"
#include "robot/threadupdaterobot.h"
#include "gui/threadupdategui.h"
#include "gui/stategui.h"

#include "easyiceconfig/EasyIce.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    Ice::CommunicatorPtr ic;

    try {
         //-----------------ICE----------------//
         ic = EasyIce::initialize(argc, argv);

         // Variables Compartidas
         // Robot -> Sensores, navegacion, actuadores
         Robot *robot = new Robot(ic);
         StateGUI* state = new StateGUI();

         ThreadUpdateRobot* thread_update_robot = new ThreadUpdateRobot(robot,state);
         thread_update_robot->start();

         ThreadUpdateGUI* thread_update_gui = new ThreadUpdateGUI(robot, state);
         thread_update_gui->start();

    } catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        exit(-1);
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        exit(-1);
    }

    app.exec();
}
