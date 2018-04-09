# -*- coding: utf-8 -*-

import sys
from docutils.nodes import image
from sensors import sensor
import numpy as np
import threading
from pyProgeo.progeo import Progeo
import cv2


class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.imageRight=np.zeros((320,240,3), np.uint8)
        self.imageLeft=np.zeros((320,240,3), np.uint8)
        self.lock = threading.Lock()

        print("Left Camera Configuration File:")
        self.camLeftP=Progeo(sys.argv[1], "CamACalibration")
        print("Rigth Camera Configuration File:")
        self.camRightP=Progeo(sys.argv[1], "CamBCalibration")

        self.done=False

        self.counter=0

    def setRightImageFiltered(self, image):
        self.lock.acquire()
        size=image.shape
        if len(size) == 2:
            image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        self.imageRight=image
        self.lock.release()


    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        size=image.shape
        if len(size) == 2:
            image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        self.imageLeft=image
        self.lock.release()

    def execute(self):
      
        #OBTENCIÓN DE IMÁGENES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()
        
        if self.done:
            return

        # Add your code here
        # pointIn=np.array([502,21,1])
        # pointInOpt=self.camLeftP.graficToOptical(pointIn)
        # point3d=self.camLeftP.backproject(pointInOpt)
        # print "Punto 3d: "
        # print point3d
        # projected1 = self.camRightP.project(point3d)
        # print "reprojected: "
        # print projected1
        #
        # print "converted:"
        # print self.camRightP.opticalToGrafic(projected1)      

        #OBTENCIÓN DE CONTORNOS
        '''grayL = cv2.cvtColor(imageLeft, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imageRight, cv2.COLOR_BGR2GRAY)
        #gaussL = cv2.GaussianBlur(imageLeft, (7, 7), 3)
        #gaussR = cv2.GaussianBlur(imageRight, (7, 7), 3)

        tL, dstL = cv2.threshold(grayL, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_TRIANGLE)
        tR, dstR = cv2.threshold(grayR, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_TRIANGLE)

        # obtener los contornos
        _, contoursL, _ = cv2.findContours(dstL, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contoursR, _ = cv2.findContours(dstR, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # dibujar los contornos
        cv2.drawContours(imageLeft, contoursL, -1, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.drawContours(imageRight, contoursR, -1, (0, 0, 255), 2, cv2.LINE_AA)

        self.setLeftImageFiltered(imageLeft)
        self.setRightImageFiltered(imageRight)'''


        myRedPoint = np.array([0, 0, 1+self.counter])
        print "Drawing Red point -> ", myRedPoint
        self.sensor.drawPoint(myRedPoint ,(255,0,0))
    
        myBluePoint = np.array([0, 50, 1+self.counter])
        self.counter += 1
        print "Drawing Blue point -> ", myBluePoint
        self.sensor.drawPoint(myBluePoint ,(0,0,255))

        lowThreshold=20
        max_lowThreshold=100
        ratio=3
        kernel_size=3

        imageLeftGray=cv2.cvtColor(imageLeft,cv2.COLOR_BGR2GRAY)
        imageRightGray=cv2.cvtColor(imageRight,cv2.COLOR_BGR2GRAY)
        leftCanny=cv2.Canny(imageLeftGray,lowThreshold, lowThreshold*ratio, kernel_size)
        rightCanny=cv2.Canny(imageRightGray,lowThreshold, lowThreshold*ratio, kernel_size)

        self.setLeftImageFiltered(leftCanny)
        self.setRightImageFiltered(rightCanny)

        '''
        lowThreshold=20
        max_lowThreshold=100
        ratio=3
        kernel_size=3



        imageLeftGray=cv2.cvtColor(imageLeft,cv2.COLOR_BGR2GRAY)
        imageRightGray=cv2.cvtColor(imageRight,cv2.COLOR_BGR2GRAY)
        leftCanny=cv2.Canny(imageLeftGray,lowThreshold, lowThreshold*ratio, kernel_size)
        rightCanny=cv2.Canny(imageRightGray,lowThreshold, lowThreshold*ratio, kernel_size)

        print "PPPPP: ", cv2.countNonZero(leftCanny)
        print leftCanny.shape
        print "size progeo: ", self.camRightP.height , ", " , self.camRightP.width
        counter=0
        if not self.done:
            size=20
            for y in range(size+1,self.camRightP.height - size - 1):
                for x in range(size+1,self.camRightP.width-size-1):
                    print 'Position: ', x ,',',y,' value:', leftCanny[y,x]

                    if leftCanny[y,x] != 0:
                        counter+=1
                        currentPoint=np.array([x,y,1])
                        currentPointOptical=self.camLeftP.graficToOptical(currentPoint)
                        print "optical: ", currentPointOptical
                        point3d=self.camLeftP.backproject(currentPointOptical)

                        print "Punto 3d: ",  point3d

                        #undo:
                        tempPoint=self.camLeftP.project(point3d)
                        print "teste: ", self.camLeftP.opticalToGrafic(tempPoint)


                        leftCamPos=self.camLeftP.getCameraPosition()
                        # newP_X = point3d[0]+1000
                        # newP_Y = ((newP_X - leftCamPos[0])/(point3d[0]-leftCamPos[0]))*(point3d[1]-leftCamPos[1]) + leftCamPos[1];
                        # newP_Z = ((newP_X - leftCamPos[0])/(point3d[0]-leftCamPos[0]))*(point3d[2]-leftCamPos[2]) + leftCamPos[2];

                        newP_X = leftCamPos[0]
                        newP_Y = leftCamPos[1]
                        newP_Z = leftCamPos[2]


                        point3d_Right=self.camRightP.project(point3d)
                        point3d_Right_Graph=self.camRightP.opticalToGrafic(point3d_Right)

                        newP_Right = self.camRightP.project(np.array([newP_X,newP_Y,newP_Z,1]))
                        newP_Right_Graph = self.camRightP.opticalToGrafic(newP_Right)

                        print "p1: ", point3d_Right_Graph
                        print "p2: ", newP_Right_Graph



                        #buscamos un path en la camara derecha.

                        leftRoi=imageLeft[y-size/2:y+size/2,x-size/2:x+size/2:]

                        offset=20
                        bestValue=99999999999
                        best=-1
                        for p in range(0,offset):
                            newX = x-p
                            if (x-p < size):
                                newX=size
                            newY = ((newX-point3d_Right_Graph[0]) /(newP_Right_Graph[0] - point3d_Right_Graph[0]))*(newP_Right_Graph[1]-point3d_Right_Graph[1]) + point3d_Right_Graph[1]
                            rightRoi=imageRight[newY-size/2:newY+size/2,newX-size/2:newX+size/2,:]
                            d=cv2.absdiff(leftRoi,rightRoi)

                            #print p,' ', np.sum(d)
                            if np.sum(d) < bestValue:
                                best=p
                                bestValue=np.sum(d)
                        newY = ((x -best-point3d_Right_Graph[0]) /(newP_Right_Graph[0] - point3d_Right_Graph[0]))*(newP_Right_Graph[1]-point3d_Right_Graph[1]) + point3d_Right_Graph[1]
                        print 'located corr at: ', x -best,',',newY
                        bestCorr=np.array([x -best, newY,1])
                        bestCorrOptical = self.camRightP.graficToOptical(bestCorr)
                        point3dRight=self.camRightP.backproject(bestCorrOptical)

                        print "1 ", leftCamPos
                        print "2 ", point3d
                        print "3 ", self.camRightP.getCameraPosition()
                        print "4 ", point3dRight


                        p1,p2=self.lineLineIntersection(leftCamPos,point3d,self.camRightP.getCameraPosition(),point3dRight)

                        meanPoint=np.array([(p1[0] + p2[0])/2, (p1[1] + p2[1])/2, (p1[2] + p2[2])/2])
                        print "POINT: ", meanPoint

                        color=imageLeft[y,x,:]
                        self.sensor.drawPoint(meanPoint,(color[0],color[1],color[2]))
                        #break

                #break



            self.setLeftImageFiltered(leftCanny)
            self.setRightImageFiltered(rightCanny)
            print "FIN -> ", counter
            self.done=True



            #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
            #self.sensor.setV(10)
            #self.sensor.setW(5)


            #SHOW THE FILTERED IMAGE ON THE GUI
            # self.setRightImageFiltered(imageRight)
            # self.setLeftImageFiltered(imageLeft)


    
    def setRightImageFiltered(self, image):
        self.lock.acquire()
        size=image.shape
        if len(size) == 2:
            image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        self.imageRight=image
        self.lock.release()


    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        size=image.shape
        if len(size) == 2:
            image=cv2.cvtColor(image,cv2.COLOR_GRAY2BGR)
        self.imageLeft=image
        self.lock.release()

    def getRightImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageRight
        self.lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageLeft
        self.lock.release()
        return tempImage
    
    def lineLineIntersection(self,p1,p2,p3,p4):
        p13 = np.empty([3,1])
        p43 = np.empty([3,1])
        p21 = np.empty([3,1])


        p13[0] = p1[0] - p3[0]
        p13[1] = p1[1] - p3[1]
        p13[2] = p1[2] - p3[2]
        p43[0] = p4[0] - p3[0]
        p43[1] = p4[1] - p3[1]
        p43[2] = p4[2] - p3[2]

        p21[0] = p2[0] - p1[0]
        p21[1] = p2[1] - p1[1]
        p21[2] = p2[2] - p1[2]


        d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2]
        d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2]
        d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2]
        d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2]
        d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2]

        denom = d2121 * d4343 - d4321 * d4321

        numer = d1343 * d4321 - d1321 * d4343

        if denom==0:
            mua=0
        else:
            mua = numer / denom
        mub = (d1343 + d4321 * (mua)) / d4343


        print denom, ", " , numer, ", ", mua, ", ", mub

        finalP1 = np.empty([3,1])
        finalP2 = np.empty([3,1])

        finalP1[0] = p1[0] + mua * p21[0];
        finalP1[1] = p1[1] + mua * p21[1];
        finalP1[2] = p1[2] + mua * p21[2];
        finalP2[0] = p3[0] + mub * p43[0];
        finalP2[1] = p3[1] + mub * p43[1];
        finalP2[2] = p3[2] + mub * p43[2];

        return finalP1,finalP2
    '''

