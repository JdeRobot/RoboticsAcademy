/*
 *
 *  Copyright (C) 1997-2011 JDERobot Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *
 *  Authors : Maikel González <m.gonzalezbai@gmail.com>
 *
 */

#include "control.h"

namespace introrob {

    Control::~Control() {
    }

    void Control::UpdateSensorsICE(Api *api) {

        api->motorVin = this->mprx->getV();
        //      if(api->motorVin!=0)
        //            std::cout << "v: " << this->mprx->getV() <<std::endl;

        api->motorWin = this->mprx->getW();
        //     if(api->motorWin!=0)
        //	   std::cout << "w: " << this->mprx->getW() <<std::endl;


        api->motorLin = this->mprx->getL();

        //////////////////////////////////////////
        api->encodersData = this->eprx->getEncodersData();
        api->pose3DData = this->p3dprx->getPose3DData();

        api->laserData = this->lprx->getLaserData();

        pthread_mutex_lock(&api->controlGui);
        api->imagesReady = FALSE;
        api->imageData1 = this->cprx1->getImageData( colorspaces::ImageRGB8::FORMAT_RGB8.get()->name );
        api->imageData2 = this->cprx2->getImageData( colorspaces::ImageRGB8::FORMAT_RGB8.get()->name );
        if (api->guiReady) {
            createImage(api);
            createImage2(api);
        }
        api->imagesReady = TRUE;
        pthread_mutex_unlock(&api->controlGui);

        api->Pose3Dencoders1 = this->p3deprx1->getPose3DEncodersData();


    }

    void Control::createImage(Api *api) {

        this->image.create(api->imageData1->description->height, api->imageData1->description->width, CV_8UC3);
	memcpy((unsigned char *) this->image.data, &(api->imageData1->pixelData[0]), this->image.cols*image.rows*3);
/*
        colorspaces::Image::FormatPtr fmt = colorspaces::Image::Format::searchFormat(api->imageData1->description->format);

        if (!fmt)
            throw "Format not supported";
*/
        api->imgBuff =
                Gdk::Pixbuf::create_from_data((const guint8*) this->image.data,
                Gdk::COLORSPACE_RGB,
                false,
                8, //this->image.depth,
                this->image.cols,
                this->image.rows,
                this->image.step);

    }

    void Control::createImage2(Api *api) {

        this->image2.create(api->imageData2->description->height, api->imageData2->description->width, CV_8UC3);
	memcpy((unsigned char *) this->image2.data, &(api->imageData2->pixelData[0]), this->image2.cols*image2.rows*3);
/*
        colorspaces::Image::FormatPtr fmt2 = colorspaces::Image::Format::searchFormat(api->imageData2->description->format);

        if (!fmt2)
            throw "Format not supported";
*/
         api->imgBuff2 =
                Gdk::Pixbuf::create_from_data((const guint8*) this->image2.data,
                Gdk::COLORSPACE_RGB,
                false,
                8, //this->image2.depth,
                this->image2.cols,
                this->image2.rows,
                this->image2.step);
    }

    // Send the actuators info to Gazebo with ICE

    void Control::SetActuatorsICE(Api *api) {

        if (api->motorWout < 5 && api->motorWout>-5)
            this->mprx->setW(0.);
        this->mprx->setW(api->motorWout);
        this->mprx->setL(api->motorLout);

        api->Pose3DmotorsData1 = new jderobot::Pose3DMotorsData();
        api->Pose3DmotorsData2 = new jderobot::Pose3DMotorsData();

        api->Pose3DmotorsData1->tilt = api->v_normalized;
        api->Pose3DmotorsData1->pan = api->w_normalized;
        api->Pose3DmotorsData2->tilt = api->v_normalized;
        api->Pose3DmotorsData2->pan = api->w_normalized;


        this->p3dmprx2->setPose3DMotorsData(api->Pose3DmotorsData2);
        this->p3dmprx1->setPose3DMotorsData(api->Pose3DmotorsData1);
        this->mprx->setV(api->motorVout);
    }

}

