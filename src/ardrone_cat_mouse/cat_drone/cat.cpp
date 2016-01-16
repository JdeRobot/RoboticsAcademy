#include "cat.h"

#define cycle 250

namespace cat {

Cat::Cat() {
    char* name = (char*) "--Ice.Config=cat.cfg";
    int argc = 1;
    char* argv[] = {name};
    std::cout << "Initializing..." << std::endl;
    ic = Ice::initialize(argc, argv);

    sensors_ = new DroneSensors(ic);

    Ice::ObjectPrx cmdvel = ic->propertyToProxy("cat.cmdvel.Proxy");
    if (cmdvel == 0)
        throw "Could not create proxy with cmdvel";
    cmdvelprx_ = jderobot::CMDVelPrx::checkedCast(cmdvel);
    if (cmdvelprx_ == 0)
        throw "Invalid proxy cat.cmdvel.Proxy";
    std::cout << "cmdvel connected" << std::endl;

    Ice::ObjectPrx ardroneextra = ic->propertyToProxy("cat.ardroneextra.Proxy");
    if (ardroneextra == 0)
        throw "Could not create proxy with ardroneextra";
    extraprx_ = jderobot::ArDroneExtraPrx::checkedCast(ardroneextra);
    if (extraprx_ == 0)
        throw "Invalid proxy cat.ardroneextra.Proxy";
    std::cout << "ardroneextra connected" << std::endl;

    /*Ice::ObjectPrx config = ic->propertyToProxy("cat.config.Proxy");
    if (config == 0)
        throw "Could not create proxy with remote config";
    configprx = jderobot::remoteConfigPrx::checkedCast(config);
    if (configprx == 0)
        throw "Invalid proxy cat.config.Proxy";
    std::cout << "config connected" << std::endl;*/

    std::string xml_file = "cat.xml";
    Parser quadrotor_parser("cat");
    config_ = new CatConfig();
    quadrotor_parser.ReadFile(xml_file, config_);
    hue_min_ = config_->GetMinHue();//0
    sat_min_ = config_->GetMinSat();//0;
    val_min_ = config_->GetMinVal();//0;
    hue_max_ = config_->GetMaxHue();//20;
    sat_max_ = config_->GetMaxSat();//255;
    val_max_ = config_->GetMaxVal();//255;

    edge_threshold_ = 40;
    roll_pitch_thr_ = 0.1;
    area_min_ = config_->GetMinArea();//1100;
    area_max_ = config_->GetMaxArea();//2000;
    lim_area_min_ = config_->GetMinAreaLimit();//100;
    lim_area_max_ = config_->GetMaxAreaLimit();//10000;
    velX_ = config_->GetVelX();//2;
    velY_ = config_->GetVelY();
    velZ_ = config_->GetVelZ();
}

Cat::~Cat() {}

/*void Cat::Config() {
    std::cout << "Setting angle: " << current_angle_ << std::endl;
    /* Set max angle corresponding to mode */
/*    int id = configprx->initConfiguration();
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
                              "<altitude_max_>200</altitude_max_>\n" <<
                              "<altitude_min_>0</altitude_min_>\n" <<
                              "<euler_angle_max_>" << current_angle_ << "</euler_angle_max_>\n" <<
                              "<control_vz_max_>0.2</control_vz_max_>\n" <<
                              "<control_yaw>" << 1050 * current_angle_ << "</control_yaw>\n" <<
                            "</ardrone>\n" <<
                          "</Configuration>\n" <<
                        "</ArDroneServer>\n";
    configprx->write(data.str(), id);
    configprx->setConfiguration(id);
}*/

void Cat::SeekAndDestroy() {
    struct timeval a, b;
    long diff;
    long totalb, totala;
    
    extraprx_->toggleCam();
    extraprx_->takeoff();

    usleep(2000000);
    int c = 0;
    while (true) {
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        Execute();
        //if (c++ <= 1)
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

void Cat::Execute() {
    // Image Processing
    jderobot::Pose3DDataPtr p3d = sensors_->GetPose3DData();
    Quaternion q(p3d->q0, p3d->q1, p3d->q2, p3d->q3);
    if (fabs(q.QuatToRoll()) > roll_pitch_thr_ || fabs(q.QuatToPitch()) > roll_pitch_thr_){
        return; // High angle
    }

    double angular_vel_ = config_->GetAngularVel();

    cv::Mat frame = sensors_->GetImage();
    cv::Mat imgTh,hsv;
	cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);
	cv::cvtColor(frame, hsv, CV_BGR2HSV);
	cv::inRange(hsv, cv::Scalar(hue_min_, sat_min_, val_min_), cv::Scalar(hue_max_, sat_max_, val_max_), imgTh);

	cv::blur(imgTh, imgTh, cv::Size(7, 7));		
	cv::Mat threshold_output;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	/// Detect edges using Threshold
	cv::threshold(imgTh, threshold_output, edge_threshold_, 255, cv::THRESH_BINARY);
	/// Find contours
	cv::findContours(threshold_output, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	/// Approximate contours to polygons + get bounding rects and circles
	std::vector<std::vector<cv::Point> > contours_poly(contours.size());
	std::vector<cv::Rect> boundRect(contours.size());
	std::vector<ObjectDetected> objects;
	ObjectDetected obj=ObjectDetected();

	for(int i = 0; i < contours.size(); i++)
	{
		cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]));
		if(boundRect[i].area()>=lim_area_min_ && boundRect[i].area()<=lim_area_max_){
			obj.setBr(boundRect[i].br());
			obj.setTl(boundRect[i].tl());
			obj.setArea(boundRect[i].area());
			obj.setY(boundRect[i].tl().y+boundRect[i].size().height/2);
			obj.setX(boundRect[i].tl().x+boundRect[i].size().width/2);
			obj.setDetected(true);
			objects.push_back(obj);
		}
	}

    // Select biggest object
	ObjectDetected object;
	if(objects.size()>0){
		object=*std::max_element(objects.begin(),objects.end());
		object.setDetected(true);
	}

    jderobot::CMDVelDataPtr cmdvel = new jderobot::CMDVelData(0, 0, 0, 0, 0, 0);
	if(object.isDetected()){
        std::cout << "mouse detected\n";
		int newX=object.getX();
		int newY=object.getY();
		int newArea=object.getArea();
        // Change coordinates
		newX = (newX-(frame.cols/2));
		newY = ((frame.cols/2)-newY);
        if (fabs(newX) > VERT_X_DEADBAND) {
            cmdvel->linearY = -1 * velY_ * newX/(frame.cols/2);
            cmdvel->angularZ = -1*angular_vel_ * newX / (frame.cols/2);
        } if (fabs(newY) > VERT_Y_DEADBAND)
            cmdvel->linearZ = velZ_ * newY/fabs(newY);///(frame.rows/2);
        if (newArea < area_min_/2)
            cmdvel->linearX = 1.5 * velX_;
        /*else if (newArea < area_min_/2)
            cmdvel->linearX = 1.5 * velX_;*/
        else if (newArea < area_min_) {
            cmdvel->linearX = velX_;
        } else if (newArea > area_max_)
            cmdvel->linearX = -0.1;
    } else {
        std::cout << "mouse not detected\n";
        cmdvel->angularZ = 2*angular_vel_;
        cmdvel->linearZ = 0.05;
    }
    cmdvelprx_->setCMDVelData(cmdvel);
}

} //cat

int main(int argc, char* argv[]) {	
    cat::Cat* sylvester = new cat::Cat();
    sylvester->SeekAndDestroy();
    return 1;
}
