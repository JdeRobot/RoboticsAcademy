#include "path_config.h"

namespace mouse {

PathConfig::PathConfig() {
    path_points_.reserve(20);
    next_unreached_ = 0;
}

PathConfig::~PathConfig() {
}

int PathConfig::get_number_beacons() const {
    return path_points_.size();
}

void PathConfig::AddInfo(const std::vector<std::string>& vbeacon) {
    std::cout << "Setting path point\n";
    std::string sbeacon = vbeacon[0];
    std::vector<std::string> tokens;
    boost::split(tokens, sbeacon, boost::is_any_of(" "));
    if (!tokens.size())
        return;
    Vector beacon(tokens.size());
    for (int i = 0; i < tokens.size(); i++)
        beacon.set_coor(i, atof(tokens.at(i).c_str()));
    path_points_.push_back(beacon);

    std::cout << "Point saved: ";
    std::cout << beacon.get_coor(0);
    for (int i = 1; i < beacon.Size(); i++)
        std::cout << ", " << beacon.get_coor(i);
    std::cout << std::endl;
}

std::vector<double> PathConfig::GetNextPoint() const{
    Vector beacon(0);
    if (next_unreached_ >= path_points_.size())
        return beacon.GetCoordinatesAsDouble();
    beacon = path_points_[next_unreached_];
    return beacon.GetCoordinatesAsDouble();
}

void PathConfig::FlagPointAsReached() {
    next_unreached_++;
}

} //mouse
