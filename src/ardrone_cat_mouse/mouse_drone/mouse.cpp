#include "mouse.h"

#define cycle 250

namespace mouse {

Mouse::Mouse() {
    char* name = (char*) "--Ice.Config=mouse.cfg";
    int argc = 1;
    char* argv[] = {name};
    std::cout << "Initializing..." << std::endl;
    ic = Ice::initialize(argc, argv);

    //IMU
	Ice::ObjectPrx baseImu = ic->propertyToProxy("mouse.pose3d.Proxy");
	if (baseImu==0)
		throw "Could not create imu proxy";
    jderobot::Pose3DPrx pose3dprx = jderobot::Pose3DPrx::checkedCast(baseImu);
	if (pose3dprx==0)
		throw "Invalid imu proxy";
    std::cout << "imu connected\n";

    Ice::ObjectPrx cmdvel = ic->propertyToProxy("mouse.cmdvel.Proxy");
    if (cmdvel == 0)
        throw "Could not create proxy with cmdvel";
    jderobot::CMDVelPrx cmdvelprx = jderobot::CMDVelPrx::checkedCast(cmdvel);
    if (cmdvelprx == 0)
        throw "Invalid proxy mouse.cmdvel.Proxy";
    std::cout << "cmdvel connected" << std::endl;

    Ice::ObjectPrx ardroneextra = ic->propertyToProxy("mouse.ardroneextra.Proxy");
    if (ardroneextra == 0)
        throw "Could not create proxy with ardroneextra";
    jderobot::ArDroneExtraPrx extraprx = jderobot::ArDroneExtraPrx::checkedCast(ardroneextra);
    if (extraprx == 0)
        throw "Invalid proxy mouse.ardroneextra.Proxy";
    std::cout << "ardroneextra connected" << std::endl;

    Ice::ObjectPrx config = ic->propertyToProxy("mouse.config.Proxy");
    if (config == 0)
        throw "Could not create proxy with remote config";
    configprx = jderobot::remoteConfigPrx::checkedCast(config);
    if (configprx == 0)
        throw "Invalid proxy mouse.config.Proxy";
    std::cout << "config connected" << std::endl;

    std::string xml_file = "path.xml";
    travel_ = new Travel(pose3dprx, cmdvelprx, extraprx, xml_file, "beacon");
    travel_->ConfigurePath();
    travel_->StartTravel();

    mouse_config_ = new MouseConfig();
    Parser quadrotor_parser("mouse");
    quadrotor_parser.ReadFile(xml_file, mouse_config_);

    status_ = Preparing;
    elapsed_time_ = 0;
    angles_[0] = mouse_config_->get_angle(0);
    angles_[1] = mouse_config_->get_angle(1);
    angles_[2] = mouse_config_->get_angle(2);
    angles_[3] = mouse_config_->get_angle(3);
    linears_[0] = mouse_config_->get_linear(0);
    linears_[1] = mouse_config_->get_linear(1);
    linears_[2] = mouse_config_->get_linear(2);
    linears_[3] = mouse_config_->get_linear(3);
    times_[0] = mouse_config_->get_time(0);
    times_[1] = mouse_config_->get_time(1);
    times_[2] = mouse_config_->get_time(2);
    times_[3] = mouse_config_->get_time(3);
    current_angle_ = 0;
    next_time_ = times_[Easy];
    //SetRunningMode();
    std::cout << "Mouse created\n";
}

Mouse::~Mouse() {}

void Mouse::SetRunningMode() {
    std::cout << "Setting angle: " << current_angle_ << std::endl;
    /* Set max angle corresponding to mode */
    int id = configprx->initConfiguration();
    std::stringstream data;
    data <<             "<ArDroneServer>\n" <<
                          "<Configuration>\n" <<
                            "<ardrone>\n" <<
                              "<default_camera>1</default_camera>\n" <<
                              "<outdoor>0</outdoor>\n" <<
                              "<max_bitrate>4000</max_bitrate>\n" <<
                              "<bitrate>1500</bitrate>\n" <<
                              "<navdata_demo>0</navdata_demo>\n" <<
                              "<flight_without_shell>0</flight_without_shell>\n" <<
                              "<altitude_max>200</altitude_max>\n" <<
                              "<altitude_min>0</altitude_min>\n" <<
                              "<euler_angle_max>" << current_angle_ << "</euler_angle_max>\n" <<
                              "<control_vz_max>0.2</control_vz_max>\n" <<
                              "<control_yaw>" << 1050 * current_angle_ << "</control_yaw>\n" <<
                            "</ardrone>\n" <<
                          "</Configuration>\n" <<
                        "</ArDroneServer>\n";
    configprx->write(data.str(), id);
    configprx->setConfiguration(id);
}

void Mouse::CheckTimesAndRunFaster() {
    if (elapsed_time_ > next_time_ && status_ < Hard) {
        status_ += 1;
        std::cout << "New level\n";
        if (status_ < Hard) {
            next_time_ = times_[status_ + 1];
        }
        current_angle_ = angles_[status_];
        //SetRunningMode();
        travel_->set_linear_vel(linears_[status_]);
        travel_->set_angular_vel(angles_[status_]);
    }
}

void Mouse::Survive() {
    struct timeval a, b;
    long diff;
    long totalb, totala;

    gettimeofday(&a, NULL);
    long offset_time = a.tv_sec * 1000000 + a.tv_usec;

    while (status_ != End) {
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;
        elapsed_time_ = totala - offset_time;

        CheckTimesAndRunFaster();
        if (travel_->NextMovement()) {
            travel_->MoveToNextPoint();
        } else {
            status_ = End;
            std::cout << "Caught\n";
        }

        //Sleep Algorithm
        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;
        if (diff < 0 || diff > cycle)
            diff = cycle;
        else
            diff = cycle - diff;

        /*Sleep Algorithm*/
        usleep(diff * 1000);
        if (diff < 33)
            usleep(33 * 1000);
        //printf("CONTROL %.15lf seconds elapsed\n", diff);
    }
}

} //mouse

int main(int argc, char* argv[]) {	
    mouse::Mouse* speedy_gonzalez = new mouse::Mouse();
    speedy_gonzalez->Survive();
    return 1;
}
