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

#include "gui.h"

namespace introrob {

    Gui::Gui(Api *api) : gtkmain(0, 0) {

        // Init some attributes
        this->api = api;
        this->yourCode_button_isPressed = false;
        press_stop_button = false;
        press_stopCodeButton = false;
        this->showCanvasWin = false;
        this->api->numlines = 0;
        this->previous_event_x = 100;
        this->previous_event_y = 100;
        this->previous_event_x_camera = 100;
        this->previous_event_y_camera = 100;
        api->guiReady = TRUE;
        //this->isFollowing = false;
        laser_box = 0;
        cameras_box = 0;
        world_box = 0;
        windowDepurate_box = 0;

        /*Init OpenGL*/
        if (!Gtk::GL::init_check(NULL, NULL)) {
            std::cerr << "Couldn't initialize GL\n";
            std::exit(1);
        }
        Gnome::Canvas::init();
        std::cout << "Loading glade\n";
        refXml = Gnome::Glade::Xml::create("./introrob.glade");
        this->gladepath = std::string("./introrob.glade");

        // GET WIDGETS & WINDOWS

        //Get windows       
        refXml->get_widget("secondarywindow", secondarywindow);
        refXml->get_widget("world_window", world_window);
        refXml->get_widget("windowLaser", windowLaser);
        refXml->get_widget("windowTeleoperate", windowTeleoperate);

        // Button, area and window canvas teleoperate robot
        refXml->get_widget("stop_button", stop_button); //canvas
        refXml->get_widget("w_canvas_teleoperate", w_canvas_teleoperate);
        refXml->get_widget("yourCodeButton", yourCodeButton);
        refXml->get_widget("stopCodeButton", stopCodeButton);
        refXml->get_widget("exitButton", exitButton);

        // Area canvas laser
        refXml->get_widget("canvas_laser", w_canvas_laser);

        // Button, area and window canvas teleoperate cameras
        refXml->get_widget("w_canvas_teleoperate_cameras", w_canvas_teleoperate_cameras);
        refXml->get_widget("centerCameras_button", centerCameras_button);

        // Camera images
        refXml->get_widget("image1", gtk_image1);
        refXml->get_widget("image2", gtk_image2);
        refXml->get_widget("pioneerCameraButton", pioneerCameraButton);

        //Buttons world
        refXml->get_widget_derived("world", this->world);
        this->world->gui = this;
        refXml->get_widget("deleteLinesButton", deleteLinesButton);

        // Image events
        refXml->get_widget("eventbox_left", eventbox_left);
        eventbox_left->add_events(Gdk::BUTTON_PRESS_MASK);
        eventbox_left->signal_button_press_event().connect(sigc::mem_fun(this, &Gui::on_left_clicked));
        refXml->get_widget("eventbox_right", eventbox_right);
        eventbox_right->add_events(Gdk::BUTTON_PRESS_MASK);
        eventbox_right->signal_button_press_event().connect(sigc::mem_fun(this, &Gui::on_right_clicked));

        // Check Buttons
        refXml->get_widget("check_cameras", check_cameras);
        refXml->get_widget("check_world", check_world);
        refXml->get_widget("check_laser", check_laser);
        refXml->get_widget("check_windowDepurate", check_windowDepurate);

        // Events
        yourCodeButton->signal_clicked().connect(sigc::mem_fun(this, &Gui::yourCodeButton_clicked));
        stopCodeButton->signal_clicked().connect(sigc::mem_fun(this, &Gui::stopCodeButton_clicked));
        exitButton->signal_clicked().connect(sigc::mem_fun(this, &Gui::exitButton_clicked));
        stop_button->signal_clicked().connect(sigc::mem_fun(this, &Gui::stop_button_clicked));
        deleteLinesButton->signal_clicked().connect(sigc::mem_fun(this, &Gui::deleteLinesButton_clicked));
        centerCameras_button->signal_clicked().connect(sigc::mem_fun(this, &Gui::stopcameras_button_clicked));
        check_cameras->signal_toggled().connect(sigc::mem_fun(this, &Gui::button_cameras_clicked));
        check_world->signal_toggled().connect(sigc::mem_fun(this, &Gui::button_world_clicked));
        check_laser->signal_toggled().connect(sigc::mem_fun(this, &Gui::button_laser_clicked));
        check_windowDepurate->signal_toggled().connect(sigc::mem_fun(this, &Gui::button_windowDepurate_clicked));
        w_canvas_teleoperate->signal_event().connect(sigc::mem_fun(this, &Gui::on_button_press_canvas_teleoperate));
        w_canvas_teleoperate_cameras->signal_event().connect(sigc::mem_fun(this, &Gui::on_button_press_canvas_teleoperate_cameras));
        pioneerCameraButton->signal_clicked().connect(sigc::mem_fun(this, &Gui::pioneerCameraButton_clicked));


        // Show or hide windows
        stopCodeButton->hide();
        windowTeleoperate->show();
        secondarywindow->show();
        windowLaser->show();

        //Calibrate cameras
        this->initCameras();

        //Init DrawAreas: Teleoperate, laser and teleoperateCameras
        w_canvas_teleoperate->signal_unrealize();
        w_canvas_teleoperate->signal_realize();
        w_canvas_teleoperate->set_child_visible(TRUE);
        w_canvas_teleoperate->add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK | Gdk::VISIBILITY_NOTIFY_MASK | Gdk::BUTTON1_MOTION_MASK);
        w_canvas_teleoperate_cameras->add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK | Gdk::VISIBILITY_NOTIFY_MASK | Gdk::BUTTON1_MOTION_MASK);

        gc_teleoperate = Gdk::GC::create(w_canvas_teleoperate->get_window());
        gc_teleoperate_cameras = Gdk::GC::create(w_canvas_teleoperate_cameras->get_window());
        w_canvas_laser->signal_unrealize();
        w_canvas_laser->signal_realize();
        w_canvas_laser->set_child_visible(TRUE);
        gc_laser = Gdk::GC::create(w_canvas_laser->get_window());

        colormap = w_canvas_laser->get_colormap();
        color_white = Gdk::Color("#FFFFFF");
        color_black = Gdk::Color("#000000");
        color_red = Gdk::Color("#FF0000");
        colormap->alloc_color(color_white);
        colormap->alloc_color(color_black);
        colormap->alloc_color(color_red);

        windowLaser->hide();
        secondarywindow->hide();

        m_image = Gdk::Pixbuf::create_from_file("myimage.png");
    }

    Gui::~Gui() {
        delete this->world;
        delete this->api;
    }

    void Gui::ShowImages(Api *api) {
        if (check_cameras) {
            pthread_mutex_lock(&api->controlGui);
            //this->updateCameras(api);
            setCamaras();
            //setCamara(*this->image2, 2);
            pthread_mutex_unlock(&api->controlGui);

            //setCamara(*api->image1, 1);
            //setCamara(*api->image2, 2);		  

        }
    }

    void Gui::updateCameras(Api *api) {

        colorspaces::Image::FormatPtr fmt1 = colorspaces::Image::Format::searchFormat(api->imageData1->description->format);
        if (!fmt1)
            throw "Format not supported";
        this->image1 = new colorspaces::Image(api->imageData1->description->width, api->imageData1->description->height, fmt1, &(api->imageData1->pixelData[0])); // Prepare the image to use with openCV

        colorspaces::Image::FormatPtr fmt2 = colorspaces::Image::Format::searchFormat(api->imageData2->description->format);
        if (!fmt2)
            throw "Format not supported";
        this->image2 = new colorspaces::Image(api->imageData2->description->width, api->imageData2->description->height, fmt2, &(api->imageData2->pixelData[0])); // Prepare the image to use with openCV

    }

    void Gui::display(Api *api) {

        this->setDestino();

        //this->Encoders2world();
        //std::cout << "Using pose not encoders" << std::endl;
        this->Pose3D2world();

        this->Laser2world();

        if (!(this->yourCode_button_isPressed)) {
            this->teleoperate();
            api->iterationControlActivated = false;

        } else {
            api->iterationControlActivated = true;
        }
        api->guiVisible = windowTeleoperate->is_visible();

        if ((!windowLaser->is_visible()) && (check_laser->get_active())) {
            check_laser->set_active(false);
        }

        if ((!secondarywindow->is_visible()) && (check_cameras->get_active())) {
            check_cameras->set_active(false);
        }

        if ((!world_window->is_visible()) && (check_world->get_active())) {
            check_world->set_active(false);
        }



        if (check_laser->get_active()) {
            this->printLaser();
        }
        if (check_cameras->get_active())
            this->teleoperateCameras();

        if (windowTeleoperate->is_visible() == false)
            exit(0);

        while (gtkmain.events_pending())
            gtkmain.iteration();

    }

    bool Gui::on_button_press_canvas_teleoperate(GdkEvent * event) {
        float event_x = event->button.x;
        float event_y = event->button.y;
        float k = 0.01;
        float p = -1;
        float v_normalized, w_normalized;
        static gboolean dragging = FALSE;

        switch (event->type) {
            case GDK_BUTTON_PRESS:
                if (event->button.button == 3) {
                    this->previous_event_x = event->button.x;
                    this->previous_event_y = event->button.y;
                }
                if (event->button.button == 1) {
                    GdkCursor *cursor;
                    cursor = gdk_cursor_new(GDK_FLEUR);
                    gdk_cursor_destroy(cursor);

                    dragging = true;
                }
                break;

            case GDK_MOTION_NOTIFY:
                if (dragging && (event->motion.state & GDK_BUTTON1_MASK)) {
                    this->previous_event_x = event_x;
                    this->previous_event_y = event_y;
                    this->teleoperate();
                }
                break;

            case GDK_BUTTON_RELEASE:
                dragging = FALSE;
                break;


            default:
                break;

        }
        v_normalized = 400 * (k * previous_event_y + p)*(-1);
        w_normalized = 20 * (k * previous_event_x + p)*(-1);

        api->setMotorV(v_normalized);
        if (w_normalized < 0.2 && w_normalized>-0.2)
            api->setMotorW(0.0);
        else
            api->setMotorW(w_normalized);



    }

    bool Gui::on_button_press_canvas_teleoperate_cameras(GdkEvent * event) {
        float event_x = event->button.x;
        float event_y = event->button.y;
        float k = 0.01;
        float p = -1;
        float v_normalized, w_normalized;
        static gboolean dragging = FALSE;
        float tilt, pan;

        switch (event->type) {
            case GDK_BUTTON_PRESS:
                if (event->button.button == 3) {
                    this->previous_event_x_camera = event->button.x;
                    this->previous_event_y_camera = event->button.y;
                }
                if (event->button.button == 1) {
                    GdkCursor *cursor;
                    cursor = gdk_cursor_new(GDK_FLEUR);
                    gdk_cursor_destroy(cursor);

                    dragging = true;
                }
                break;

            case GDK_MOTION_NOTIFY:
                if (dragging && (event->motion.state & GDK_BUTTON1_MASK)) {
                    this->previous_event_x_camera = event_x;
                    this->previous_event_y_camera = event_y;
                    this->teleoperateCameras();
                }
                break;

            case GDK_BUTTON_RELEASE:
                dragging = FALSE;
                break;


            default:
                break;

        }

        widget2Pose3Dmotors(this->previous_event_x_camera, this->previous_event_y_camera, &tilt, &pan);

        v_normalized = 400 * (k * previous_event_y_camera + p)*(-1);
        w_normalized = 20 * (k * previous_event_x_camera + p)*(1);
        api->v_normalized = tilt;
        api->w_normalized = pan;



    }

    void Gui::widget2Pose3Dmotors(int x, int y, float* tilt, float* pan) {

        float tilt_y = api->Pose3Dencoders1->maxTilt;
        float tilt_x = (api->Pose3Dencoders1->minTilt - tilt_y) / 200;
        float pan_y = api->Pose3Dencoders1->minPan;
        float pan_x = (api->Pose3Dencoders1->maxPan - pan_y) / 200;

        *pan = x * pan_x + pan_y;
        *tilt = y * tilt_x + tilt_y;

    }

    void Gui::teleoperate() {
        int i;
        gc_teleoperate->set_foreground(color_black);
        w_canvas_teleoperate->get_window()->draw_rectangle(gc_teleoperate, true, 0, 0, 200, 200);


        w_canvas_teleoperate->get_window()->draw_pixbuf(m_image,
                0, 0, this->previous_event_x - 12, this->previous_event_y - 12,
                m_image->get_width(),
                m_image->get_height(),
                Gdk::RGB_DITHER_NONE,
                -1, -1);

        gc_teleoperate->set_foreground(color_red);
        w_canvas_teleoperate->get_window()->draw_line(gc_teleoperate, 0, previous_event_y, 200, previous_event_y);
        w_canvas_teleoperate->get_window()->draw_line(gc_teleoperate, previous_event_x, 0, previous_event_x, 200);


        gc_teleoperate->set_foreground(color_white);
        w_canvas_teleoperate->get_window()->draw_line(gc_teleoperate, 100, 0, 100, 200);
        w_canvas_teleoperate->get_window()->draw_line(gc_teleoperate, 0, 100, 200, 100);
    }

    void Gui::teleoperateCameras() {
        int i;
        float k = 0.0526;
        float p = 0;
        float v_normalized, w_normalized;
        float pan, tilt;


        gc_teleoperate_cameras->set_foreground(color_black);
        w_canvas_teleoperate_cameras->get_window()->draw_rectangle(gc_teleoperate_cameras, true, 0, 0, 200, 200);

        w_canvas_teleoperate_cameras->get_window()->draw_pixbuf(m_image,
                0, 0, this->previous_event_x_camera - 12, this->previous_event_y_camera - 12,
                m_image->get_width(),
                m_image->get_height(),
                Gdk::RGB_DITHER_NONE,
                -1, -1);

        pan = api->Pose3Dencoders1->pan;
        tilt = api->Pose3Dencoders1->tilt;
        gc_teleoperate_cameras->set_foreground(color_red);


        float tilt_x = 200 / (api->Pose3Dencoders1->minTilt + api->Pose3Dencoders1->minTilt);
        float tilt_y = api->Pose3Dencoders1->minTilt*tilt_x;

        float pan_x = 200 / (api->Pose3Dencoders1->maxPan + api->Pose3Dencoders1->maxPan);
        float pan_y = api->Pose3Dencoders1->maxPan*pan_x;


        w_normalized = (api->Pose3Dencoders1->pan * pan_x + pan_y);
        v_normalized = (api->Pose3Dencoders1->tilt * tilt_x + tilt_y);



        w_canvas_teleoperate_cameras->get_window()->draw_line(gc_teleoperate_cameras, 0, v_normalized, 200, v_normalized);
        w_canvas_teleoperate_cameras->get_window()->draw_line(gc_teleoperate_cameras, w_normalized, 0, w_normalized, 200);


        gc_teleoperate_cameras->set_foreground(color_white);
        w_canvas_teleoperate_cameras->get_window()->draw_line(gc_teleoperate_cameras, 100, 0, 100, 200);
        w_canvas_teleoperate_cameras->get_window()->draw_line(gc_teleoperate_cameras, 0, 100, 200, 100);


    }

    void Gui::printLaser() {
        jderobot::LaserDataPtr laser;
        laser = api->getLaserData();
        int vectorLaser[180];
        int i;

        gc_laser->set_foreground(color_black);
        w_canvas_laser->get_window()->draw_rectangle(gc_laser, true, 0, 0, 360, 180);
        gc_laser->set_foreground(color_red);
        w_canvas_laser->get_window()->draw_line(gc_laser, 180, 165, 180, 180);
        gc_laser->set_foreground(color_white);


        for (i = laser->numLaser - 1; i > 0; i--) {
            w_canvas_laser->get_window()->draw_line(gc_laser, 180 + ((laser->distanceData[i] / 45)*(cos((i) * PI / 180))), 180 - ((laser->distanceData[i] / 45)*(sin((i) * PI / 180))), 180 + ((laser->distanceData[i + 1] / 45)*(cos((i + 1) * PI / 180))), 180 - ((laser->distanceData[i + 1] / 45)*(sin((i + 1) * PI / 180))));
        }
    }

    float Gui::getAspectRatio(int width1, int height1, int width2, int height2) {
        float arw, arh;
        arw = (float)width1 / (float)width2;
        arh = (float)height1 / (float)height2;
        if (arw < arh) {
            return arh;
        }
        return arw;
    }

    void Gui::setCamaras() {
		// Variables para el cálculo de la relación de aspecto idóneo
        if (api->imagesReady) {
            gtk_image1->clear();
            if (gtk_image1->get_width() < api->imgBuff->get_width() ||
                    gtk_image1->get_height() < api->imgBuff->get_height()) {
                // Cambiamos la resolución manteniendo la relación de aspecto
                image1_ratio = Gui::getAspectRatio(api->imgBuff->get_width(), api->imgBuff->get_height(),
                        gtk_image1->get_width(), gtk_image1->get_height());
                api->imgBuff = api->imgBuff->scale_simple(api->imgBuff->get_width() / image1_ratio,
                        api->imgBuff->get_height() / image1_ratio, Gdk::INTERP_BILINEAR);
            } else {
                image1_ratio = 1;
            }
            gtk_image1->set(api->imgBuff);
            gtk_image2->clear();
            if (gtk_image2->get_width() < api->imgBuff2->get_width() ||
                    gtk_image2->get_height() < api->imgBuff2->get_height()) {
                // Cambiamos la resolución manteniendo la relación de aspecto
                image2_ratio = Gui::getAspectRatio(api->imgBuff2->get_width(), api->imgBuff2->get_height(),
                        gtk_image2->get_width(), gtk_image2->get_height());
                api->imgBuff2 = api->imgBuff2->scale_simple(api->imgBuff2->get_width() / image2_ratio,
                api->imgBuff2->get_height() / image2_ratio, Gdk::INTERP_BILINEAR);
            } else {
                image2_ratio = 1;
            }
            gtk_image2->set(api->imgBuff2);
        }

    }

    void Gui::isVisible() {
    }

    void Gui::yourCodeButton_clicked() {

        yourCodeButton->hide();
        stopCodeButton->show();
        //canvas_control->hide();
        stop_button->hide();
        yourCode_button_isPressed = true;
    }

    void Gui::stopCodeButton_clicked() {

        yourCodeButton->show();
        stopCodeButton->hide();
        //canvas_control->show();
        stop_button->show();
        press_stop_button = true;
        resetAPI(this->api);
        this->yourCode_button_isPressed = false;
    }

    void Gui::exitButton_clicked() {

        api->guiVisible = false;
        windowTeleoperate->hide();
        exit(0);
    }

    void Gui::stop_button_clicked() {
        this->previous_event_x = 100;
        this->previous_event_y = 100;
        press_stop_button = true;
        resetAPI(this->api);
    }

    void Gui::stopcameras_button_clicked() {

        this->previous_event_y_camera = 100;
        this->previous_event_x_camera = 100;
        api->v_normalized = 0; //28*(k*v+p);
        api->w_normalized = 0; //-45*(k*w+p);

    }

    void Gui::resetAPI(Api *api) {

        api->setMotorV(0);
        api->setMotorW(0);
        api->setMotorL(0);
    }

    bool Gui::on_right_clicked(GdkEventButton * event) {
        gint x, y;
        gdk_window_at_pointer(&x, &y);

        api->x_click_cameraright = x;
        api->y_click_cameraright = y;

        int offsetx = (gtk_image2->get_width() - gtk_image2->get_pixbuf()->get_width()) / 2;
        pixB.x = (x - offsetx) * image2_ratio;
        int offsety = (gtk_image2->get_height() - gtk_image2->get_pixbuf()->get_height()) / 2;
        pixB.y = (y - offsety) * image2_ratio;
        pixB.h = 1.0;
        printf("click en camera derecha, punto %f,%f\n", pixB.x, pixB.y);
        //this->calculate_projection_line(pixB,2);
        return true;
    }

    bool Gui::on_left_clicked(GdkEventButton * event) {
        gint x, y;
        gdk_window_at_pointer(&x, &y);

        api->x_click_cameraleft = x;
        api->y_click_cameraleft = y;

        int offsetx = (gtk_image1->get_width() - gtk_image1->get_pixbuf()->get_width()) / 2;
        pixA.x = (x - offsetx) * image1_ratio;
        int offsety = (gtk_image1->get_height() - gtk_image1->get_pixbuf()->get_height()) / 2;
        pixA.y = (y - offsety) * image1_ratio;
        pixA.h = 1.0;
        printf("click en camera izquierda, punto %f,%f\n", pixA.x, pixA.y);
        //this->calculate_projection_line(pixA,1);
        return true;
    }

    void Gui::deleteLinesButton_clicked() {
        this->resetLines();
    }

    void Gui::button_cameras_clicked() {
        if (!check_cameras->get_active()) {
            secondarywindow->hide();

        } else
            secondarywindow->show();
    }

    void Gui::button_world_clicked() {
        if (!check_world->get_active())
            world_window->hide();
        else
            world_window->show();
    }

    void Gui::button_laser_clicked() {
        if (!check_laser->get_active()) {
            windowLaser->hide();
        } else {
            windowLaser->show();
        }
    }

    void Gui::button_windowDepurate_clicked() {
        if (check_windowDepurate->get_active()) {
            cvNamedWindow("DebuggingWin", CV_WINDOW_AUTOSIZE);
            api->showImage = true;
        } else {
            api->showImage = false;
            cvDestroyWindow("DebuggingWin");
        }
    }
///////////////////////
    void Gui::Encoders2world() {
        this->world->roboty = this->api->encodersData->roboty;
        this->world->robotx = this->api->encodersData->robotx;
        this->world->robottheta = this->api->encodersData->robottheta;

        //std::cout << "Encoders:"<< this->world->robotx << " "<< this->world->roboty << " " << this->world->robottheta << " " <<std::endl;
    }

    void Gui::Pose3D2world() {
        this->world->roboty = this->api->pose3DData->y;
        this->world->robotx = this->api->pose3DData->x;

        /*
         * The following section calculates the angle required to properly visualize the pioneer robot
         */


        double magnitude,w,x,y,z,squ,sqx,sqy,sqz;

        magnitude = sqrt(this->api->pose3DData->q0 * this->api->pose3DData->q0 + this->api->pose3DData->q1 * this->api->pose3DData->q1 + this->api->pose3DData->q2 * this->api->pose3DData->q2 + this->api->pose3DData->q3 * this->api->pose3DData->q3);

        w = this->api->pose3DData->q0 / magnitude;
        x = this->api->pose3DData->q1 / magnitude;
        y = this->api->pose3DData->q2 / magnitude;
        z = this->api->pose3DData->q3 / magnitude;

        squ = w * w;
        sqx = x * x;
        sqy = y * y;
        sqz = z * z;

        double angle;

        angle = atan2( 2 * (x * y + w * z), squ + sqx - sqy - sqz) * 180.0 / PI;

        if(angle < 0)
        {
            angle += 360.0;
        }

        this->world->robottheta = angle;

        //std::cout << "Pose3D:"<< this->world->robotx << " "<< this->world->roboty << " " << this->world->robottheta << " " <<std::endl;
    }

    void Gui::Laser2world() {
        int k;
        this->world->distanceData.clear();
        this->world->numLasers = 0;
        for (k = 0; k < this->api->laserData->numLaser; k++) {

            this->world->distanceData.push_back(this->api->laserData->distanceData[k]);
            this->world->numLasers++;
        }
    }

    void Gui::setDestino() {

        this->api->destino.x = this->world->destino.x;
        this->api->destino.y = this->world->destino.y;
        //printf ("myPoint = [%f, %f]\n", this->api->destino.x, this->api->destino.y);

    }

    void Gui::calculate_projection_line(HPoint2D pix, int idcamera) {
        HPoint3D pdraw3D;

        if (idcamera == 1) {
            this->get3DPositionZ(&myCamA, pdraw3D, pix, 0.); // 5000 son los 5 metros a los que está la pared.

            this->add_line(myCamA.position.X, myCamA.position.Y, myCamA.position.Z, (float) pdraw3D.X, (float) pdraw3D.Y, (float) pdraw3D.Z, idcamera);
            printf("click en camera, punto %f,%f\n", pix.x, pix.y);

        } else {
            this->get3DPositionZ(&myCamB, pdraw3D, pix, 0.); // 5000 son los 5 metros a los que está la pared.
            this->add_line(myCamB.position.X, myCamB.position.Y, myCamB.position.Z, (float) pdraw3D.X, (float) pdraw3D.Y, (float) pdraw3D.Z, idcamera);
        }
        printf("click en camera, punto %f,%f\n", pix.x, pix.y);
    }

    void Gui::initCameras() {
        // Init camera 1
        xmlReader(&myCamA, "cameras/calibA.xml");
        //        camera *mycameraA = new camera("cameras/calibA");
        //        myCamA = mycameraA->readConfig();

        // Init camera 2
        //        camera *mycameraB = new camera("cameras/calibB");
        //        myCamB = mycameraB->readConfig();
        xmlReader(&myCamB, "cameras/calibB.xml");
    }

    void Gui::get3DPositionZ(TPinHoleCamera * camera, HPoint3D &res, HPoint2D in, float Z = 0.0) {
        HPoint2D p2d;
        HPoint3D p3d;
        float x, y, z;
        float xfinal, yfinal, zfinal;

        x = camera->position.X;
        y = camera->position.Y;
        z = camera->position.Z;

        p2d.x = in.x;
        p2d.y = in.y;
        p2d.h = in.h;

        this->pixel2optical(camera, &p2d);
        backproject(&p3d, p2d, *camera);

        /*Check division by zero*/
        if ((p3d.Z - z) == 0.0) {
            res.H = 0.0;
            return;
        }

        zfinal = Z;

        /*Linear equation (X-x)/(p3d.X-x) = (Y-y)/(p3d.Y-y) = (Z-z)/(p3d.Z-z)*/
        xfinal = x + (p3d.X - x)*(zfinal - z) / (p3d.Z - z);
        yfinal = y + (p3d.Y - y)*(zfinal - z) / (p3d.Z - z);

        res.X = xfinal;
        res.Y = yfinal;
        res.Z = zfinal;
        res.H = 1.0;
    }

    void Gui::add_line(float x0, float y0, float z0, float x1, float y1, float z1, int color) {
        if (this->api->numlines < MAX_LINES) {
            this->api->extra_lines[this->api->numlines][0] = x0;
            this->api->extra_lines[this->api->numlines][1] = y0;
            this->api->extra_lines[this->api->numlines][2] = z0;
            this->api->extra_lines[this->api->numlines][3] = 0;
            this->api->extra_lines[this->api->numlines][4] = x1;
            this->api->extra_lines[this->api->numlines][5] = y1;
            this->api->extra_lines[this->api->numlines][6] = z1;
            this->api->extra_lines[this->api->numlines][7] = 0;
            this->api->extra_lines[this->api->numlines][8] = color;
            this->api->numlines++;
        } else {
            printf("error, too many lines in the world\n");

        }

    }

    void Gui::pixel2optical(TPinHoleCamera *cam, HPoint2D *p) {
        float aux;
        int height;

        height = cam->rows;
        aux = p->x;
        p->x = height - 1 - p->y;
        p->y = aux;
    }

    void Gui::resetLines() {
        this->api->numlines = 0;
    }

    void Gui::callGraphicsOnApi() {

        this->api->RunGraphicsAlgorithm();

    }
//////////////////////
    void Gui::pioneerCameraButton_clicked() {

        //this->world->setToPioneerCamera(this->api->encodersData->robottheta);
        this->world->setToPioneerCamera(this->world->robottheta);
    }



} // namespace    

