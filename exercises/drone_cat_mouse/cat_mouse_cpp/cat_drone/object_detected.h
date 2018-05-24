#ifndef OBJECT_DETECTED_H
#define OBJECT_DETECTED_H
#include <iostream>
#include <opencv2/core/core.hpp>
class ObjectDetected
{
	public:
		ObjectDetected();
		~ObjectDetected();
		int getX(){return x;}
		int getY(){return y;}
		int getArea(){return area;}
		bool isDetected(){return detected;}
		void setX(int val){x=val;}
		void setY(int val){y=val;}
		void setArea(int val){area=val;}
		void setDetected(bool val){detected=val;}
		void setBr(cv::Point val){br=val;}
		cv::Point getBr(){return br;}
		void setTl(cv::Point val){tl=val;}		
		cv::Point getTl(){return tl;}		
		
		bool operator>(ObjectDetected obj);
		bool operator<(ObjectDetected obj);
		bool operator==(ObjectDetected obj);
		ObjectDetected operator=(ObjectDetected obj);		
	private:
		friend std::ostream& operator<<(std::ostream&, const ObjectDetected&);
		cv::Point br;
		cv::Point tl;
		int x;
		int y;
		int area;
		bool detected;
};
#endif // OBJECT_DETECTED_H
