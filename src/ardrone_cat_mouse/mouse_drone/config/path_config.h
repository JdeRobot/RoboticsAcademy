#ifndef PATHCONFIG_H
#define PATHCONFIG_H

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include "drone_config.h"

namespace mouse {

class PathConfig : public DroneConfig {
public:
	PathConfig();
    ~PathConfig();

    virtual void AddInfo(const std::vector<std::string>&);
    int get_number_beacons() const;
    std::vector<double> GetNextPoint() const;
    void FlagPointAsReached();
protected:
    class Vector {
    public:
        Vector(int size) {
            coor_ = new std::vector<double>(size);
        }
        int Size() const {
            return coor_->size();
        }
        void set_coor(int i, double c) {
            coor_->at(i);
            (*coor_)[i] = c;
        }
        double get_coor(int i) {
            try {
            return coor_->at(i);
            } catch (std::out_of_range o) {
                return 0;
            }
        }
        std::vector<double> GetCoordinatesAsDouble() {
            std::vector<double> aux = *coor_;
            return aux;
        }
        Vector operator+(const Vector p) {
            Vector returned(p.Size());
            for (int i = 0; i < coor_->size(); i++)
                (*returned.coor_)[i] = ((*coor_)[i] + (*p.coor_)[i]);
            return returned;
        }
        Vector operator-(const Vector p) {
            Vector returned(p.Size());
            for (int i = 0; i < coor_->size(); i++)
                (*returned.coor_)[i] = ((*coor_)[i] - (*p.coor_)[i]);
            return returned;
        }
        Vector operator/(double d) {
            Vector returned(Size());
            for (int i = 0; i < coor_->size(); i++)
                (*returned.coor_)[i] = ((*coor_)[i] / d);
            return returned;
        }
        Vector operator*(double d) {
            Vector returned(Size());
            for (int i = 0; i < coor_->size(); i++)
                (*returned.coor_)[i] = ((*coor_)[i] * d);
            return returned;
        }
        bool operator==(const Vector v) {
            if (Size() != v.Size()) return false;
            bool returned = true;
            for (int i = 0; i < coor_->size(); i++)
                returned = returned & (*coor_)[i] == (*v.coor_)[i];
            return returned;
        }
        bool operator!=(const Vector v) {
            if (Size() != v.Size()) return true;
            bool returned = true;
            for (int i = 0; i < coor_->size(); i++)
                returned = returned & (*coor_)[i] != (*v.coor_)[i];
            return returned;
        }
    protected:
        std::vector<double> *coor_;
    }; // Vector
    class Vector3 : public Vector {
    public:
        Vector3(float x, float y, float z) : Vector(3){
            coor_->push_back(x);
            coor_->push_back(y);
            coor_->push_back(z);
        }
        double get_x() {
            return (*coor_)[0];
        }
        double get_y() {
            return (*coor_)[1];
        }
        double get_z() {
            return (*coor_)[2];
        }
        void set_x(double x) {
            (*coor_)[0] = x;
        }
        void set_y(double y) {
            (*coor_)[1] = y;
        }
        void set_z(double z) {
            (*coor_)[2] = z;
        }
    }; // Vector3
    typedef std::vector<Vector> PointsVector;
    PointsVector path_points_;
    int next_unreached_;
};

}

#endif //PATHCONFIG_H
