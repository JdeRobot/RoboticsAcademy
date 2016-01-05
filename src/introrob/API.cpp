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
 *g
 *  Authors : Maikel González <m.gonzalezbai@gmail.com>,
 *
 */

#include "API.h"

#define TWOPI 6.283185308
#define GRAPHIC_TO_OPTICAL_X(x,y) (SIFNTSC_ROWS-1-y)
#define GRAPHIC_TO_OPTICAL_Y(x,y) (x)    
#define OPTICAL_TO_GRAPHIC_X(x,y) (y)
#define OPTICAL_TO_GRAPHIC_Y(x,y) (SIFNTSC_ROWS-1-x)

namespace introrob {

    float Api::getMotorV() {
        return this->motorVin;
    }

    float Api::getMotorW() {
        return this->motorWin;
    }

    float Api::getMotorL() {
        return this->motorLin;
    }

    jderobot::LaserDataPtr Api::getLaserData() {
        return this->laserData;
    }

    int Api::getNumLasers() {
        return this->laserData->numLaser;
    }

    jderobot::IntSeq Api::getDistancesLaser() {
        return this->laserData->distanceData;
    }

    //////////////////////////

    jderobot::EncodersDataPtr Api::getEncodersData() {
        return this->encodersData;
    }

    jderobot::Pose3DDataPtr Api::getPose3DData() {
        return this->pose3DData;
    }

    cv::Mat Api::getImageCamera1() {

	cv::Mat cvResultado = this->image1.clone();
	return cvResultado;
    }

    cv::Mat Api::getImageCamera2() {

	cv::Mat cvResultado = this->image2.clone();
	return cvResultado;
    }

    void Api::setMotorV(float motorV) {
        this->motorVout = motorV;
    }

    void Api::setMotorW(float motorW) {
        this->motorWout = motorW;
    }

    void Api::setMotorL(float motorL) {
        this->motorLout = motorL;

    }

    /*
    void Api::setPTEncoders(double pan, double tilt, int cameraId){

    api->PTmotorsData1 = new jderobot::PTMotorsData ();
    api->PTmotorsData2 = new jderobot::PTMotorsData ();

    if(cameraId==1){

    this->PTmotorsData1->latitude=pan;
    this->PTmotorsData1->longitude=tilt;
    }
    else{
    this->PTmotorsData2->latitude=pan;
    this->PTmotorsData2->longitude=tilt;
    }
    }
     */

    int Api::graficas2opticas(double pointX, double pointY, HPoint2D *punto2D) {




        punto2D->x = GRAPHIC_TO_OPTICAL_X(pointX, pointY);
        punto2D->y = GRAPHIC_TO_OPTICAL_Y(pointX, pointY);
        punto2D->h = 1.0000;



        return 0;
    }

    int Api::opticas2graficas(HPoint2D punto2D, double *pointX, double *pointY) {

        *pointX = OPTICAL_TO_GRAPHIC_X(punto2D.x, punto2D.y);
        *pointY = OPTICAL_TO_GRAPHIC_Y(punto2D.x, punto2D.y);

        return 0;
    }

    void Api::imageCameras2openCV() {
        pthread_mutex_lock(&this->controlGui);

	// cv::Mat processing
	this->imageCameraRight.create(imageData2->description->height, imageData2->description->width, CV_8UC3);
	memcpy((unsigned char *) this->imageCameraRight.data, &(imageData2->pixelData[0]), this->imageCameraRight.cols*imageCameraRight.rows*3);

	this->imageCameraLeft.create(imageData1->description->height, imageData1->description->width, CV_8UC3);
	memcpy((unsigned char *) this->imageCameraLeft.data, &(imageData1->pixelData[0]), this->imageCameraLeft.cols*imageCameraLeft.rows*3);

        /*
        colorspaces::Image::FormatPtr fmt1 = colorspaces::Image::Format::searchFormat(imageData1->description->format);
        if (!fmt1)
            throw "Format not supported";
        image1 = new colorspaces::Image(imageData1->description->width, imageData1->description->height, fmt1, &(imageData1->pixelData[0])); // Prepare the image to use with openCV

        colorspaces::Image::FormatPtr fmt2 = colorspaces::Image::Format::searchFormat(imageData2->description->format);
        if (!fmt2)
            throw "Format not supported";
        image2 = new colorspaces::Image(imageData2->description->width, imageData2->description->height, fmt2, &(imageData2->pixelData[0])); // Prepare the image to use with openCV
         */
        pthread_mutex_unlock(&this->controlGui);
          
    }

    int Api::absolutas2relativas(CvPoint3D32f in, CvPoint3D32f *out) {
        if (out != NULL) {
            CvPoint3D32f myPoint;

            //this->controller->getPosition(&myPoint);
            this->robotx = encodersData->robotx;
            this->roboty = encodersData->roboty;
            this->robottheta = encodersData->robottheta;

            (*out).x = in.x * cos(this->robottheta * DEGTORAD) + in.y * sin(this->robottheta * DEGTORAD) -
                    this->robotx * cos(this->robottheta * DEGTORAD) - this->roboty * sin(this->robottheta * DEGTORAD);
            (*out).y = in.y * cos(this->robottheta * DEGTORAD) - in.x * sin(this->robottheta * DEGTORAD) -
                    this->roboty * cos(this->robottheta * DEGTORAD) + this->robotx * sin(this->robottheta * DEGTORAD);
            return 0;
        }
        return 1;
    }

    int Api::relativas2absolutas(CvPoint3D32f in, CvPoint3D32f *out) {
        if (out != NULL) {
            CvPoint3D32f myPoint;

            this->robotx = encodersData->robotx;
            this->roboty = encodersData->roboty;
            this->robottheta = encodersData->robottheta;

            (*out).x = in.x * cos(this->robottheta * DEGTORAD) - in.y * sin(this->robottheta * DEGTORAD) + this->robotx;
            (*out).y = in.y * cos(this->robottheta * DEGTORAD) + in.x * sin(this->robottheta * DEGTORAD) + this->roboty;
            (*out).z = in.z;
            return 0;
        }
        return 1;
    }

    int Api::pintaSegmento(CvPoint3D32f a, CvPoint3D32f b, CvPoint3D32f color) {
        /* OJO mundo de coordenadas OpenGL está en decímetros, por compatibilidad con la plantilla OpenGL
        del robotPioneer. El factor SCALE marca la relación entre las coordenadas en milímetros de a,b
        y los homólogos en el mundo OpenGL */

        glColor3f(color.x, color.y, color.z);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        v3f(a.x / SCALE, a.y / SCALE, a.z / SCALE);
        v3f(b.x / SCALE, b.y / SCALE, b.z / SCALE);
        glEnd();
        return 1;
    }

    void Api::drawSphere(CvPoint3D32f a, CvPoint3D32f color) {
        glColor3f(color.x, color.y, color.z);
        glPushMatrix();
        glTranslatef(a.x / SCALE, a.y / SCALE, a.z / SCALE);
        glutSolidSphere(1, 20, 20);
        glPopMatrix();
        glEnd();
    }

    int Api::pintaDestino(CvPoint3D32f a, CvPoint3D32f b, CvPoint3D32f color) {
        /* OJO mundo de coordenadas OpenGL está en decímetros, por compatibilidad con la plantilla OpenGL
        del robotPioneer. El factor SCALE marca la relación entre las coordenadas en milímetros de a,b
        y los homólogos en el mundo OpenGL */

        glColor3f(color.x, color.y, color.z);
        glLineWidth(2.0f);

        glBegin(GL_TRIANGLES);
        glVertex3f((b.x / SCALE), (b.y / SCALE) + 3, b.z / SCALE);
        glVertex3f((b.x / SCALE) + 1.5, (b.y / SCALE) + 1.5, b.z / SCALE);
        glVertex3f((b.x / SCALE) - 1.5, (b.y / SCALE) + 1.5, b.z / SCALE);

        glVertex3f((b.x / SCALE) + 3, (b.y / SCALE), b.z / SCALE);
        glVertex3f((b.x / SCALE) + 1.5, (b.y / SCALE) + 1.5, b.z / SCALE);
        glVertex3f((b.x / SCALE) + 1.5, (b.y / SCALE) - 1.5, b.z / SCALE);

        glVertex3f((b.x / SCALE), (b.y / SCALE) - 3, b.z / SCALE);
        glVertex3f((b.x / SCALE) + 1.5, (b.y / SCALE) - 1.5, b.z / SCALE);
        glVertex3f((b.x / SCALE) - 1.5, (b.y / SCALE) - 1.5, b.z / SCALE);

        glVertex3f((b.x / SCALE) - 3, (b.y / SCALE), b.z / SCALE);
        glVertex3f((b.x / SCALE) - 1.5, (b.y / SCALE) + 1.5, b.z / SCALE);
        glVertex3f((b.x / SCALE) - 1.5, (b.y / SCALE) - 1.5, b.z / SCALE);


        glEnd();
        return 1;
    }

    void Api::drawProjectionLines() {

        for (int i = 0; i < numlines; i++) {
            CvPoint3D32f a, b, aa, ba;
            CvPoint3D32f color;
            a.x = extra_lines[i][0];
            a.y = extra_lines[i][1];
            a.z = extra_lines[i][2];
            b.x = extra_lines[i][4];
            b.y = extra_lines[i][5];
            b.z = extra_lines[i][6];

            if (extra_lines[i][8] == 1) {
                color.x = 1.;
                color.y = 0.;
                color.z = 0.;
            } else {
                color.x = 0.;
                color.y = 0.;
                color.z = 1.;
            }
            this->pintaSegmento(a, b, color);
        }

    }

    CvPoint2D32f Api::getDestino() {
        return this->destino;
    }

    void Api::showMyImage() {
        int i, j;

        imshow("DebuggingWin", this->imageCameraLeft);

    }

    Api::~Api() {
    }

}
