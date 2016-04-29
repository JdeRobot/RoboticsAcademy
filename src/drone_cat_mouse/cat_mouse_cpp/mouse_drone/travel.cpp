#include "travel.h"

namespace mouse {

Travel::Travel( jderobot::Pose3DPrx pose3dprx,
                jderobot::CMDVelPrx cmdvelprx,
                jderobot::ArDroneExtraPrx extraprx,
                std::string path_file,
                std::string xml_key) {
    this->pose3dprx_ = pose3dprx;
    this->cmdvelprx = cmdvelprx;
    this->extraprx = extraprx;
    this->path_file_ = path_file;
    this->path_config_ = new PathConfig();
    this->travel_config_ = new TravelConfig();
    this->file_keyword_ = xml_key;
    Parser quadrotor_parser("quadrotor");
    quadrotor_parser.ReadFile(path_file_, travel_config_);
}

Travel::~Travel() {
    delete path_config_;
    delete travel_config_;
}

bool Travel::GoTo(double x, double y, double z) {
    float distance;
    jderobot::Pose3DDataPtr drone_pose = pose3dprx_->getPose3DData();
    //distance
    float dist_xy = sqrt(   pow(y - drone_pose->y, 2) +
                            pow(x - drone_pose->x, 2));
    float dist_z = z - drone_pose->z;
    distance = sqrt(pow(dist_xy, 2) + pow(dist_z, 2));
    //std::cout << "Distance: " << distance << std::endl;
    jderobot::CMDVelDataPtr drone_vel = new jderobot::CMDVelData();
    drone_vel->angularX = drone_vel->angularY = 0;
    if (fabs(distance) < travel_config_->get_dist_thr()) {
        drone_vel->linearX = drone_vel->linearY = drone_vel->linearZ = 0;
        drone_vel->angularX = drone_vel->angularY = drone_vel->angularZ = 0;
        cmdvelprx->setCMDVelData(drone_vel);
        return true;
    }
    //yaw angle and angular velocity
    float phi = atan(   (y - drone_pose->y) /
                        (x - drone_pose->x));
    if (drone_pose->x > x) {
        if (drone_pose->y < y)
            phi += M_PI;
        else
            phi -= M_PI;
    }
    Quaternion drone_quat(  drone_pose->q0, drone_pose->q1,
                            drone_pose->q2, drone_pose->q3);
    float drone_yaw = drone_quat.QuatToYaw();
    float alpha = phi - drone_yaw;
    float yaw_thr = travel_config_->get_yaw_thr();
    //float vel = travel_config_->get_max_lin_vel();
    //float ang_vel = travel_config_->get_max_ang_vel();
    if (fabs(alpha) > yaw_thr & fabs(dist_xy) > travel_config_->get_xydist_thr()){
        drone_vel->angularZ = alpha > 0 ? ang_vel_ : -1 * ang_vel_;
        if (drone_yaw * phi < 0 && fabs(drone_yaw - phi) > M_PI) {
            drone_vel->angularZ = -1 * drone_vel->angularZ;
        }
    }else {
        drone_vel->angularZ = 0;
        alpha = 0;
    }
    //std::cout << "Angle: " << alpha << std::endl;

    //linear velocity
    if (fabs(dist_xy) > travel_config_->get_xydist_thr()) {
        drone_vel->linearX = lin_vel_ * cos(alpha);
        drone_vel->linearY = lin_vel_ * sin(alpha);
    } else
        drone_vel->linearX = drone_vel->linearY = 0;
    if (fabs(dist_z) > travel_config_->get_zdist_thr()) {
        drone_vel->linearZ = dist_z > 0 ? lin_vel_ : -1 * lin_vel_;
    } else
        drone_vel->linearZ = 0;
    cmdvelprx->setCMDVelData(drone_vel);
    return false;
}

bool Travel::GoTo(double x, double y) {
    jderobot::Pose3DDataPtr drone_pose = pose3dprx_->getPose3DData();
    return GoTo(x, y, drone_pose->z);
}

bool Travel::MoveToNextPoint() {
    bool arrived;
    try {
        beacon_.at(2);
        arrived = GoTo(beacon_[0], beacon_[1], beacon_[2]);
    } catch (std::out_of_range o) {
        arrived = GoTo(beacon_[0], beacon_[1]);
    }
    if (arrived) {
        path_config_->FlagPointAsReached();
        beacon_ = path_config_->GetNextPoint();
        PrintAndAddOffset();
        return true;
    }
    return false;
}

bool Travel::NextMovement() {
    if (!beacon_.size()) return false;
    return true;
}

void Travel::ConfigurePath() {
    Parser path_parser(file_keyword_);
    path_parser.ReadFile(path_file_, path_config_);
    beacon_ = path_config_->GetNextPoint();
}

void Travel::set_linear_vel(double v) {
    this->lin_vel_ = v;
}

void Travel::set_angular_vel(double v) {
    this->ang_vel_ = v;
}

void Travel::PrintAndAddOffset() {
    std::cout << "Next beacon: " << beacon_[0];
    for (int i = 1; i < beacon_.size(); i++)
        std::cout << ", " << beacon_[i];
    std::cout << std::endl;
    try {
        beacon_.at(2);
        beacon_[2] += travel_config_->get_zoffset();
        std::cout << "GoTo 3D point\n";
    } catch (std::out_of_range o) {
        std::cout << "GoTo 2D point\n";
    }
}

void Travel::StartTravel() {
    extraprx->takeoff();
    if (!NextMovement()) return;
    PrintAndAddOffset();
}

} //mouse
