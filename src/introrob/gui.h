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
 *  Authors : Maikel González <m.gonzalezbai@gmail.com>
 *
 */

#ifndef INTROROB_GUI_H
#define INTROROB_GUI_H

#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <string>
#include <iostream>
#include <gtkmm.h>
#include <gtkglmm.h>
#include <gdkglmm.h>
#include <libglademm.h>
#include <IceUtil/Thread.h>
#include <IceUtil/Time.h>
#include <jderobot/camera.h>
#include <pthread.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <gtkmm/drawingarea.h>
#include <gdkmm/pixbuf.h>
#include "API.h"
//#include "camera.h"
#include <progeo/progeo.h>
#include <libgnomecanvasmm.h> 
#include "drawarea.h"

#define NUM_LASERS 180
#define MAX_LINES 200
#define SCALE 100
#define PI 3.14159265

namespace introrob {
    class CanvasWin;
    class Api;
    class DrawArea;

    class Gui {
    public:

        Gui(Api *api);
        virtual ~Gui();



        //Public Methods
        void callGraphicsOnApi();
        void isVisible();
        void display(Api *api);
        void ShowImages(Api *api);
        void updateCameras(Api *api);
        void resetAPI(Api *api);
        void setDestino();
        void prepare2draw(cv::Mat image);
        void calculate_projection_line(HPoint2D pix, int idcamera);
        void get3DPositionZ(TPinHoleCamera * camera, HPoint3D &res, HPoint2D in, float Z);
        void add_line(float x0, float y0, float z0, float x1, float y1, float z1, int color);
        void pixel2optical(TPinHoleCamera *cam, HPoint2D *p);
        void resetLines();




    private:
        // CANVAS
        Gtk::DrawingArea *w_canvas_laser;
        Gtk::DrawingArea *w_canvas_teleoperate;
        Gtk::DrawingArea *w_canvas_teleoperate_cameras;
        Glib::RefPtr<Gdk::Pixbuf> m_image;
        colorspaces::Image* image1; // Image camera1 processed to manipulate with openCV
        colorspaces::Image* image2; // Image camera2 processed to manipulate with openCV
        introrob::DrawArea* world;
        Glib::RefPtr<Gdk::GC> gc_laser;
        Glib::RefPtr<Gdk::GC> gc_teleoperate;
        Glib::RefPtr<Gdk::GC> gc_teleoperate_cameras;
        Glib::RefPtr<Gdk::Colormap> colormap;
        Gdk::Color color_white;
        Gdk::Color color_black;
        Gdk::Color color_red;


        // Windows
        Gtk::Window * canvaswindow;
        Gtk::Window * windowTeleoperate;
        Gtk::Window * windowLaser;
        Gtk::Window *secondarywindow;
        Gtk::Window *world_window;
        Gtk::Window * depurate_window;

        // Private Methods
        void initCameras();
        float getAspectRatio(int width1, int height1, int width2, int height2);
        void setCamaras();
        void yourCodeButton_clicked();
        void stopCodeButton_clicked();
        void exitButton_clicked();
        void canvas_button_toggled();
        void stop_button_clicked();
        void stopcameras_button_clicked();
        bool on_left_clicked(GdkEventButton * event);
        bool on_right_clicked(GdkEventButton * event);
        void deleteLinesButton_clicked();
        void Encoders2world();
        void Pose3D2world();
        void Laser2world();
        void printLaser();
        void teleoperate();
        void teleoperateCameras();
        bool on_button_press_canvas_teleoperate(GdkEvent* event);
        bool on_button_press_canvas_teleoperate_cameras(GdkEvent* event);
        bool on_canvas_event(GdkEvent * event, Gnome::Canvas::Item * item);
        void pioneerCameraButton_clicked();

        //Checks if the button has been clicked
        void button_cameras_clicked();
        void button_world_clicked();
        void button_laser_clicked();
        void button_windowDepurate_clicked();


        // Buttons	
        bool showCanvasWin;
        Gtk::CheckButton * canvas_button;
        bool press_stop_button;
        Gtk::Button *stop_button;
        Gtk::Button *yourCodeButton;
        bool yourCode_button_isPressed;
        Gtk::Button *stopCodeButton;
        bool press_stopCodeButton;
        Gtk::Button *exitButton;
        Gtk::Button *deleteLinesButton;
        Gtk::Button *centerCameras_button;
        Gtk::Button *pioneerCameraButton;

        //Check Buttons
        Gtk::CheckButton * check_cameras;
        Gtk::CheckButton * check_world;
        Gtk::CheckButton * check_laser;
        Gtk::CheckButton * check_windowDepurate;

        // Events
        Gtk::EventBox * eventbox_left;
        Gtk::EventBox * eventbox_right;


        // Cameras
        Gtk::Image *gtk_image1;
        Gtk::Image *gtk_image2;
        float image1_ratio, image2_ratio; // Size ratio: original image <-> GTK widget
        TPinHoleCamera myCamA, myCamB;
        HPoint2D pixA; // for pixel projection
        HPoint2D pixB; // for pixel projection	


        // Others			    
        Gtk::Main gtkmain;
        Glib::RefPtr<Gnome::Glade::Xml> refXml;
        std::string gladepath;
        introrob::Api *api;
        float extra_lines[MAX_LINES][9];
        int previous_event_x, previous_event_y, previous_event_y_camera, previous_event_x_camera;
        void widget2Pose3Dmotors(int x, int y, float* tilt, float* pan);

        float robotx;
        float roboty;
        float robottheta;

        bool laser_box;
        bool cameras_box;
        bool world_box;
        bool windowDepurate_box;

        bool isFollowing;







    }; //class
}//namespace
#endif //INTROROB_GUI_H
