#include "control.h"
#include "API.h"
#include "gui.h"
#define cycle_control 50 //miliseconds
#define cycle_gui 100 //miliseconds


//Global Memory
introrob::Api *api;

void *showGui(void*) {

    struct timeval a, b;
    long totalb, totala;
    int cont = 0;
    long diff;
    introrob::Gui *gui;

    gui = new introrob::Gui(api);


    while (true) {
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        gui->ShowImages(api);
        gui->display(api);


        //Sleep Algorithm
        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;
        if (diff < 0 || diff > cycle_gui)
            diff = cycle_gui;
        else
            diff = cycle_gui - diff;

        /*Sleep Algorithm*/
        usleep(diff * 1000);
        if (diff < 33)
            usleep(33 * 1000);
        //printf("GUI %.30lf seconds elapsed, %d\n", diff, cont);
        cont++;

    }
}

int main(int argc, char** argv) {
    pthread_t thr_gui;

    introrob::Control *control;

    int status;
    Ice::CommunicatorPtr ic;
    struct timeval a, b;
    long diff;
    long totalb, totala;
    bool guiActivated = 1;
    bool controlActivated = 0;

    //---------------- INPUT ARGUMENTS ---------------//
    /*   
    if (argc!=3){
    printf("\n");
    printf("USE: ./introrob --Ice.Config=introrob.cfg OPTION\n");      
    printf("    -G to show the GUI\n");
    printf("    -C to run IterationControl\n");
    }

    if((argc==3)&&(!strcmp(argv[2],"-G"))){
    guiActivated=1;
    controlActivated=0;
    }
     */
    if ((argc == 3) && (!strcmp(argv[2], "--nogui"))) {
        controlActivated = 1;
        guiActivated = 0;
    }

    api = new introrob::Api();
    //api->showImage=true;
    if ((argc == 3) && (!strcmp(argv[2], "-d"))) {

        api->showImage = true;
    }

    //----------------END INPUT ARGUMENTS -----------------//



    control = new introrob::Control();
    //control->iterationControlActivated=false;
    pthread_mutex_init(&api->controlGui, NULL);


    try {

        //-----------------ICE----------------//
        ic = Ice::initialize(argc, argv);

        // Contact to MOTORS interface
        Ice::ObjectPrx baseMotors = ic->propertyToProxy("introrob.Motors.Proxy");
        if (0 == baseMotors)
            throw "Could not create proxy with motors";
        // Cast to motors
        control->mprx = jderobot::MotorsPrx::checkedCast(baseMotors);
        if (0 == control->mprx)
            throw "Invalid proxy introrob.Motors.Proxy";

        // Get driver camera
        Ice::ObjectPrx camara1 = ic->propertyToProxy("introrob.Camera1.Proxy");
        if (0 == camara1)
            throw "Could not create proxy to camera1 server";

        // cast to CameraPrx
        control->cprx1 = jderobot::CameraPrx::checkedCast(camara1);
        if (0 == control->cprx1)
            throw "Invalid proxy";

        // Get driver camera
        Ice::ObjectPrx camara2 = ic->propertyToProxy("introrob.Camera2.Proxy");
        if (0 == camara2)
            throw "Could not create proxy to camera2 server";

        // cast to CameraPrx
        control->cprx2 = jderobot::CameraPrx::checkedCast(camara2);
        if (0 == control->cprx2)
            throw "Invalid proxy";

        // Contact to ENCODERS interface
        Ice::ObjectPrx baseEncoders = ic->propertyToProxy("introrob.Encoders.Proxy");
        if (0 == baseEncoders)
            throw "Could not create proxy with encoders";

        // Cast to encoders
        control->eprx = jderobot::EncodersPrx::checkedCast(baseEncoders);
        if (0 == control->eprx)
            throw "Invalid proxy introrob.Encoders.Proxy";

        // Contact to POSE3D interface
        Ice::ObjectPrx basePose3D = ic->propertyToProxy("introrob.Pose3D.Proxy");
        if (0 == basePose3D)
            throw "Could not create proxy with pose3d";

        // Cast to pose3d
        control->p3dprx = jderobot::Pose3DPrx::checkedCast(basePose3D);
        if (0 == control->p3dprx)
            throw "Invalid proxy introrob.Pose3D.Proxy";

        // Contact to LASER interface
        Ice::ObjectPrx baseLaser = ic->propertyToProxy("introrob.Laser.Proxy");
        if (0 == baseLaser)
            throw "Could not create proxy with laser";

        // Cast to laser
        control->lprx = jderobot::LaserPrx::checkedCast(baseLaser);
        if (0 == control->lprx)
            throw "Invalid proxy introrob.Laser.Proxy";

        // Contact to Pose3dEncoders interface
        Ice::ObjectPrx pose3dencoders2 = ic->propertyToProxy("introrob.Pose3Dencoders2.Proxy");
        if (0 == pose3dencoders2)
            throw "Could not create proxy with encoders";

        // Cast to encoders
        control->p3deprx2 = jderobot::Pose3DEncodersPrx::checkedCast(pose3dencoders2);
        if (0 == control->p3deprx2)
            throw "Invalid proxy introrob.Pose3Dencoders2.Proxy";

        // Contact to Pose3dEncoders interface
        Ice::ObjectPrx pose3dencoders1 = ic->propertyToProxy("introrob.Pose3Dencoders1.Proxy");
        if (0 == pose3dencoders1)
            throw "Could not create proxy with encoders";

        // Cast to encoders
        control->p3deprx1 = jderobot::Pose3DEncodersPrx::checkedCast(pose3dencoders1);
        if (0 == control->p3deprx1)
            throw "Invalid proxy introrob.Pose3Dencoders1.Proxy";

        // Contact to Pose3dEncoders interface
        Ice::ObjectPrx pose3dmotors2 = ic->propertyToProxy("introrob.Pose3Dmotors2.Proxy");
        if (0 == pose3dmotors2)
            throw "Could not create proxy with encoders";

        // Cast to encoders
        control->p3dmprx2 = jderobot::Pose3DMotorsPrx::checkedCast(pose3dmotors2);
        if (0 == control->p3dmprx2)
            throw "Invalid proxy introrob.Pose3Dencoders2.Proxy";

        // Contact to Pose3dEncoders interface
        Ice::ObjectPrx pose3dmotors1 = ic->propertyToProxy("introrob.Pose3Dmotors1.Proxy");
        if (0 == pose3dmotors1)
            throw "Could not create proxy with encoders";

        // Cast to encoders
        control->p3dmprx1 = jderobot::Pose3DMotorsPrx::checkedCast(pose3dmotors1);
        if (0 == control->p3dmprx1)
            throw "Invalid proxy introrob.Pose3Dencoders1.Proxy";


        //-----------------END ICE----------------//

        //****************************** Processing the Control ******************************///
        api->guiVisible = true;
        api->sentido = 10;
        api->accion = 0;
        api->guiReady = FALSE;
        api->imagesReady = FALSE;
        control->UpdateSensorsICE(api);

        if (guiActivated) {
            pthread_create(&thr_gui, NULL, &showGui, NULL);
        }

        while (api->guiVisible) {
            gettimeofday(&a, NULL);
            totala = a.tv_sec * 1000000 + a.tv_usec;

            control->UpdateSensorsICE(api); // Update sensors
            if (controlActivated || api->iterationControlActivated) {
                api->RunNavigationAlgorithm();
                if (api->showImage)
                    api->showMyImage();
            }
            control->SetActuatorsICE(api); // Set actuators


            //Sleep Algorithm
            gettimeofday(&b, NULL);
            totalb = b.tv_sec * 1000000 + b.tv_usec;
            diff = (totalb - totala) / 1000;
            if (diff < 0 || diff > cycle_control)
                diff = cycle_control;
            else
                diff = cycle_control - diff;

            /*Sleep Algorithm*/
            usleep(diff * 1000);
            if (diff < 33)
                usleep(33 * 1000);
            //printf("CONTROL %.15lf seconds elapsed\n", diff);

        }

        //****************************** END Processing the Control ******************************///
        if (guiActivated)
            pthread_join(thr_gui, NULL);
    } catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        status = 1;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        status = 1;
    }




    if (ic)
        ic->destroy();
    return 0;
}

