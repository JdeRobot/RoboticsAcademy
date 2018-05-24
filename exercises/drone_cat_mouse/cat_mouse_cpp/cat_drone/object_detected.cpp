#include "object_detected.h"

ObjectDetected::ObjectDetected()
{
	this->x=0;
	this->y=0;
	this->area=0;
	this->detected=false;
	this->br=cv::Point(0,0);
	this->tl=cv::Point(0,0);	
}

ObjectDetected::~ObjectDetected()
{

}

std::ostream& operator<<(std::ostream &strm, const ObjectDetected &obj) {
	if(obj.detected){
		return strm << "[Detected] x: " << obj.x << " y: " << obj.y << " area: " << obj.area;
	}else{
		return strm << "[No detected]";
	}
}

bool ObjectDetected::operator>(ObjectDetected obj){
	if(area>obj.area){
		return true;
	}else{
		return false;
	}
}

bool ObjectDetected::operator<(ObjectDetected obj){
	if(area<obj.area){
		return true;
	}else{
		return false;
	}
}

bool ObjectDetected::operator==(ObjectDetected obj){
	if(area==obj.area){
		return true;
	}else{
		return false;
	}
}

ObjectDetected ObjectDetected::operator=(ObjectDetected obj){
	area=obj.getArea();
	x=obj.getX();
	y=obj.getY();
	detected=obj.isDetected();
	br=obj.getBr();
	tl=obj.getTl();	
	return *this;
}
