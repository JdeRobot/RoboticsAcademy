/*
 *  Copyright (C) 1997-2011 JDERobot Developers Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *  Authors : Maikel González <m.gonzalezbai@gmail.com>,
 *
 */


#ifndef INTROROB_API_H
#define INTROROB_API_H



//#include "camera.h"
#include <progeo/progeo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <jderobot/camera.h>
#include <jderobot/motors.h>
#include <jderobot/ptmotors.h>
#include <jderobot/laser.h>
#include <jderobot/encoders.h>
#include <jderobot/pose3d.h>
#include <jderobot/ptencoders.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <jderobot/pose3dmotors.h>
#include <jderobot/pose3dencoders.h>
#include <pthread.h>
#include "gui.h"


#define MAX_LINES 200



namespace introrob {

    class Api {
    public:
        virtual ~Api();

        pthread_mutex_t controlGui;

        //FUNCTIONS
        void RunNavigationAlgorithm();
        void RunGraphicsAlgorithm();
        void showMyImage();

        int graficas2opticas(double pointX, double pointY, HPoint2D *punto2D);
        int opticas2graficas(HPoint2D punto2D, double *pointX, double *pointY);
        // GETS
        float getMotorV();
        float getMotorW();
        float getMotorL();
        jderobot::LaserDataPtr getLaserData();
        int getNumLasers();
        jderobot::IntSeq getDistancesLaser();
        jderobot::EncodersDataPtr getEncodersData();
        jderobot::Pose3DDataPtr getPose3DData();
        cv::Mat getImageCamera1();
        cv::Mat getImageCamera2();

        //SETS
        void setMotorV(float motorV);
        void setMotorW(float motorW);
        void setMotorL(float motorL);
        void setPTEncoders(double pan, double tilt, int cameraId);

        /*Shared Memory -- ignored by students*/
        float motorVout;
        float motorWout;
        float motorLout;
        float motorVin;
        float motorWin;
        float motorLin;
        float v_normalized; // Used to teleoperate cameras (latitude)
        float w_normalized; // Used to teleoperate cameras (longitude)
        jderobot::EncodersDataPtr encodersData;
        jderobot::Pose3DDataPtr pose3DData;
        jderobot::LaserDataPtr laserData;
        jderobot::ImageDataPtr imageData1; // Contains the image info
        jderobot::ImageDataPtr imageData2; // Contains the image info
        jderobot::PTEncodersDataPtr PTencodersData1;
        jderobot::PTEncodersDataPtr PTecondersData2;
        jderobot::PTMotorsData* PTmotorsData1;
        jderobot::PTMotorsData* PTmotorsData2;
        jderobot::PTMotorsDataPtr PTmotors1;
        jderobot::PTMotorsDataPtr PTmotors2;
        jderobot::Pose3DEncodersDataPtr Pose3Dencoders1;
        jderobot::Pose3DEncodersDataPtr Pose3Dencoders2;
        jderobot::Pose3DMotorsData* Pose3DmotorsData1;
        jderobot::Pose3DMotorsData* Pose3DmotorsData2;
        cv::Mat image1; // Image camera1 processed to manipulate with openCV
        cv::Mat image2; // Image camera2 processed to manipulate with openCV
        bool guiVisible;
        bool iterationControlActivated;
        //Variables used in NavigationAlgorithm
        CvPoint2D32f destino;
        CvPoint2D32f myPosition;
        int sentido;
        int accion;
        bool segmentoPintado;
        int numlines;
        float extra_lines[MAX_LINES][9];
        cv::Mat imageOnCamera;
        bool showImage;
        gint x_click_cameraleft, y_click_cameraleft, x_click_cameraright, y_click_cameraright;
        TPinHoleCamera myCamA, myCamB;
        Glib::RefPtr<Gdk::Pixbuf> imgBuff2, imgBuff;
        bool imagesReady;
        bool guiReady;
        cv::Mat imageCameraLeft;
        cv::Mat imageCameraRight;

    private:
        //Variables used in NavigationAlgorithm
        cv::Mat imageCamera1;
        cv::Mat imageCamera2;

        struct XYZ {
            double x;
            double y;
            double z;
        };

        double robotx;
        double roboty;
        double robottheta;
        //CvPoint3D32f color;
        //Functions used in NavigationAlgorithm
        void imageCameras2openCV();
        /* Formato estructura color RGB => color.x = R; color.y = G; color.z B) */
        int pintaSegmento(CvPoint3D32f a, CvPoint3D32f b, CvPoint3D32f color);
        int pintaDestino(CvPoint3D32f a, CvPoint3D32f b, CvPoint3D32f color);

        /* Calcula la posicion relativa respecto del robot de un punto absoluto.
        El robot se encuentra en robotx, roboty con orientacion robotheta respecto
        al sistema de referencia absoluto */
        int absolutas2relativas(CvPoint3D32f in, CvPoint3D32f *out);

        /* Calcula la posicion absoluta de un punto expresado en el sistema de
        coordenadas solidario al robot. El robot se encuentra en robotx, roboty
        con orientacion robottheta respecto al sistema de referencia absoluto */
        int relativas2absolutas(CvPoint3D32f in, CvPoint3D32f *out);

        void drawSphere(CvPoint3D32f a, CvPoint3D32f color);

        void drawProjectionLines();
        CvPoint2D32f getDestino();


    }; //class
} // namespace
#endif /*INTROROB_Control_H*/
