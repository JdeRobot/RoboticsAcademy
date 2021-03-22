import rospy
from sensor_msgs.msg import Image as ImageROS
import yaml
import threading
from math import pi as PI
import cv2
from cv_bridge import CvBridge, CvBridgeError


MAXRANGE = 8 #max length received from imageD
MINRANGE = 0

def imageMsg2Image(img, bridge):

    image = Image()

    image.width = img.width
    image.height = img.height
    image.format = "RGB8"
    image.timeStamp = img.header.stamp.secs + (img.header.stamp.nsecs *1e-9)
    cv_image=0
    if (img.encoding[-2:] == "C1"):
        gray_img_buff = bridge.imgmsg_to_cv2(img, img.encoding)
        cv_image  = depthToRGB8(gray_img_buff, img.encoding)
    else:
        cv_image = bridge.imgmsg_to_cv2(img, "rgb8")
    image.data = cv_image
    return image

import numpy as np

class Image:

    def __init__(self):

        self.height = 3  # Image height [pixels]
        self.width = 3  # Image width [pixels]
        self.timeStamp = 0 # Time stamp [s] */
        self.format = "" # Image format string (RGB8, BGR,...)
        self.data = np.zeros((self.height, self.width, 3), np.uint8) # The image data itself
        self.data.shape = self.height, self.width, 3


    def __str__(self):
        s = "Image: {\n   height: " + str(self.height) + "\n   width: " + str(self.width)
        s = s + "\n   format: " + self.format + "\n   timeStamp: " + str(self.timeStamp) 
        s = s + "\n   data: " + str(self.data) + "\n}"
        return s 


class ListenerParameters:
    def __init__(self,configFile,cam):

        f = open(configFile, "r")
        cfg = yaml.load(f)
        #starting comm
        #jdrc= comm.init(cfg, '3DReconstruction')
        #ic = jdrc.getIc()
        #properties = ic.getProperties()

        data = cfg["3DReconstruction"][cam]["data"]
        print(data)

        self.K=np.array([data["K"][0],data["K"][1],data["K"][2],data["K"][4], data["K"][5],data["K"][6],data["K"][8],data["K"][9],data["K"][10]],dtype=np.double).reshape(3,3)
        self.RT=np.array([data["RT"][0],data["RT"][1],data["RT"][2],data["RT"][3], data["RT"][4],data["RT"][5],data["RT"][6],data["RT"][7],data["RT"][8],data["RT"][9],data["RT"][10],data["RT"][11],0,0,0,1],dtype=np.double).reshape(4,4)
        self.width=data["Size"][0]
        self.height=data["Size"][1]


    def backproject(self, point2d):
        myin_h=self.K[0,0]
        myin_x=point2d[0]*self.K[0,0]/point2d[2]
        myin_y=point2d[1]*self.K[1,1]/point2d[2]

        ik11=(1./self.K[0,0])
        ik12=-self.K[0,1]/(self.K[0,0]*self.K[1,1])
        ik13=(self.K[0,1]*self.K[1,2]-self.K[0,2]*self.K[1,1])/(self.K[1,1]*self.K[0,0])
        ik21=0.
        ik22=(1./self.K[1,1])
        ik23=-(self.K[1,2]/self.K[1,1])
        ik31=0.
        ik32=0.
        ik33=1.

        a1=ik11*myin_x+ik12*myin_y+ik13*myin_h
        a2=ik21*myin_x+ik22*myin_y+ik23*myin_h
        a3=ik31*myin_x+ik32*myin_y+ik33*myin_h
        a4=1.

        ir11=self.RT[0,0]
        ir12=self.RT[1,0]
        ir13=self.RT[2,0]
        ir14=0.
        ir21=self.RT[0,1]
        ir22=self.RT[1,1]
        ir23=self.RT[2,1]
        ir24=0.
        ir31=self.RT[0,2]
        ir32=self.RT[1,2]
        ir33=self.RT[2,2]
        ir34=0.
        ir41=0.
        ir42=0.
        ir43=0.
        ir44=1.

        b1=ir11*a1+ir12*a2+ir13*a3+ir14*a4
        b2=ir21*a1+ir22*a2+ir23*a3+ir24*a4
        b3=ir31*a1+ir32*a2+ir33*a3+ir34*a4
        b4=ir41*a1+ir42*a2+ir43*a3+ir44*a4

        it11=1.
        it12=0.
        it13=0.
        it14=-self.RT[2,3]
        it21=0.; it22=1.; it23=0.; it24=self.RT[1,3];
        it31=0.; it32=0.; it33=1.; it34=-self.RT[0,3];
        it41=0.; it42=0.; it43=0.; it44=1.;

        outPoint = np.array([0,0,0,1])
        outPoint[0]=it11*b1+it12*b2+it13*b3+it14*b4;
        outPoint[1]=it21*b1+it22*b2+it23*b3+it24*b4;
        outPoint[2]=it31*b1+it32*b2+it33*b3+it34*b4;
        outPoint[3]=it41*b1+it42*b2+it43*b3+it44*b4;
        return outPoint

    def backproject2(self, point2d):
        iK = inv(self.K)
        Pi=np.array([point2d[0]/point2d[2],point2d[1]/point2d[2],1.0],dtype=np.double).reshape(3,1)
        a=np.dot(iK,Pi)
        aH=np.array([a[0],a[1],a[2],1],dtype=np.double)
        RT2=self.RT.copy()
        RT2[0,3] = 0
        RT2[1,3] = 0
        RT2[2,3] = 0
        RT2[3,3] = 1
        b = np.dot(np.transpose(RT2),aH)
        translate = np.identity(4,dtype=np.double)
        translate[0,3] = self.RT[0,3]/self.RT[3,3];
        translate[1,3] = self.RT[1,3]/self.RT[3,3];
        translate[2,3] = self.RT[2,3]/self.RT[3,3];
        b=np.dot(translate,b)
        outPoint = np.array([b[0]/b[3],b[1]/b[3],b[2]/b[3],1])
        return outPoint

    def project(self,point3d):

        a1=self.RT[0,0]*point3d[0]+self.RT[0,1]*point3d[1]+self.RT[0,2]*point3d[2]+self.RT[0,3]*point3d[3]
        a2=self.RT[1,0]*point3d[0]+self.RT[1,1]*point3d[1]+self.RT[1,2]*point3d[2]+self.RT[1,3]*point3d[3]
        a3=self.RT[2,0]*point3d[0]+self.RT[2,1]*point3d[1]+self.RT[2,2]*point3d[2]+self.RT[2,3]*point3d[3]
        a4=self.RT[3,0]*point3d[0]+self.RT[3,1]*point3d[1]+self.RT[3,2]*point3d[2]+self.RT[3,3]*point3d[3]
        out_x=self.K[0,0]*a1+self.K[0,1]*a2+self.K[0,2]*a3;
        out_y=self.K[1,0]*a1+self.K[1,1]*a2+self.K[1,2]*a3;
        out_h=self.K[2,0]*a1+self.K[2,1]*a2+self.K[2,2]*a3;

        if out_h!=0.:
            out_x=out_x/out_h;
            out_y=out_y/out_h;
            out_h=1.;
        return np.array([out_x,out_y,out_h])


    def project2(self,point3d):
        a1=self.RT[0,0]*point3d[0] + self.RT[0,1]*point3d[1] + self.RT[0,2]*point3d[2] - self.RT[2,3]
        a2=self.RT[1,0]*point3d[0] + self.RT[1,1]*point3d[1] + self.RT[1,2]*point3d[2] + self.RT[1,3]
        a3=self.RT[2,0]*point3d[0] + self.RT[2,1]*point3d[1] + self.RT[2,2]*point3d[2] - self.RT[0,3]
        aP= np.array([a1,a2,a3],dtype=np.double)
        # a = self.RT.dot(point3d)
        # print ("aproject")
        # print (a)
        # aP = np.array([a[0]/a[3],a[1]/a[3],a[2]/a[3]],dtype=np.double)
        p = self.K.dot(aP)
        outPoint = np.array([p[0]/p[2],p[1]/p[2],1.0]);
        return outPoint

    def graficToOptical(self,point2d):
        x = point2d[0]
        y = point2d[1]
        point = np.array([self.height-1-y,x, point2d[2]])
        return point
             
    def opticalToGrafic(self,point2d):
        x = point2d[0]
        y = point2d[1]
        point = np.array([y,self.height - 1 - x, point2d[2]])
        return point

    def getCameraPosition(self):
        return np.array([-self.RT[2,3],self.RT[1,3],-self.RT[0,3]])

class ListenerCamera:
 
    def __init__(self, topic):
        
        self.topic = topic
        self.data = Image()
        self.sub = None
        self.lock = threading.Lock()

        self.bridge = CvBridge()
        self.start()
 
    def __callback (self, img):

        image = imageMsg2Image(img, self.bridge)

        self.lock.acquire()
        self.data = image
        self.lock.release()
        
    def stop(self):

        self.sub.unregister()

    def start (self):
 
        self.sub = rospy.Subscriber(self.topic, ImageROS, self.__callback)
        
    def getImage(self):

        self.lock.acquire()
        image = self.data
        self.lock.release()
        
        return image

    def hasproxy (self):

        return hasattr(self,"sub") and self.sub


