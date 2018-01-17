import threading
import time
from datetime import datetime
import cv2
import numpy as np

time_cycle = 80
numVuelta=50;
wSearch=0
timerW=10;
initTime=0
initialTime=0
yanterior=0
xanterior=0
yanteriorTot=0
xanteriorTot=0
m=0
x_img=0
y_img=0
landed=0
turnland=0
numIteracionesOrange=0
numIteracionesGreen=0


class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra
        self.input_image=None
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def setImageFiltered(self, image):
        self.lock.acquire()
        self.input_image=image
        self.lock.release()

    def getImageFiltered(self):
        self.lock.acquire()
        tempImage=self.input_image
        self.lock.release()
        return tempImage

    def centroImagen(self, input_image,hsv):
        print("CALCULA EL CENTRO DE LA IMAGEN")
        m=input_image+1-1
        lower_img = np.array([0,0,0], dtype=np.uint8)
        upper_img = np.array([255,255,255], dtype=np.uint8)
        centroimg = cv2.inRange(hsv, lower_img, upper_img)
        momentsimg=cv2.moments(centroimg)
        areaimg= momentsimg['m00']
        global x_img
        global y_img
        x_img = int(momentsimg['m10']/momentsimg['m00'])
        y_img = int(momentsimg['m01']/momentsimg['m00'])



    def center(self,show_image,maskRGBOrange,maskRGBGreen):
        f = []
        i=0
        imgray2 = cv2.cvtColor(maskRGBOrange,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray2,255,255,255)
        _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(c) for c in contours]
        for extension in areas:
            if extension > 4000:
                img = np.zeros((y_img*2,x_img*2,3), np.uint8)
                actual = contours[i]
                approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)
                cv2.drawContours(img,[actual],0,(0,30,0),12)
                f.append(img)
                i=i+1


        i=0
        imgray3 = cv2.cvtColor(maskRGBGreen,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(imgray3,255,255,255)
        _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        areas = [cv2.contourArea(c) for c in contours]
        for extension in areas:
           if extension > 4000:
               img = np.zeros((y_img*2,x_img*2,3), np.uint8)
               actual = contours[i]
               approx = cv2.approxPolyDP(actual,0.05*cv2.arcLength(actual,True),True)
               cv2.drawContours(img,[actual],0,(0,30,0),12)
               f.append(img)
               i=i+1

        kernel = np.ones((5,5),np.uint8)
        show_image2=show_image+1-1
        if(len(f)>0):
            f[0] = cv2.dilate(f[0],kernel,iterations = 4)
            show_image2=f[0]
            for k in range(len(f)-1):
                f[k+1] = cv2.dilate(f[k+1],kernel,iterations = 4)
                show_image2=show_image2+f[k+1]

        return show_image2,f

    def printAndCoord(self, show_image2,show_image,f):
                lower_green = np.array([0,80,0], dtype=np.uint8)
                upper_green = np.array([0, 255,0], dtype=np.uint8)
                maskSHI = cv2.inRange(show_image2, lower_green, upper_green)
                show_image2 = cv2.bitwise_and(show_image2,show_image2, mask= maskSHI)
                compare_image = np.zeros((y_img*2,x_img*2,3), np.uint8)
                diff_total = cv2.absdiff(compare_image, show_image2)
                imagen_gris = cv2.cvtColor(diff_total, cv2.COLOR_BGR2GRAY)
                _,contours,_ = cv2.findContours(imagen_gris,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

                positionXarr=[]
                positionYarr=[]
                positionX=-20
                positionY=-20
                for c in contours:
                    if(cv2.contourArea(c) >= 0):
                        posicion_x,posicion_y,ancho,alto = cv2.boundingRect(c)
                        cv2.rectangle(show_image,(posicion_x,posicion_y),(posicion_x+ancho,posicion_y+alto),(0,0,255),2)

                        positionX= (posicion_x+posicion_x+ancho)/2
                        positionY= (posicion_y+posicion_y+ancho)/2

                        positionXarr.append(positionX)
                        positionYarr.append(positionY)


                show_image2=f[0]
                for k in range(len(f)-1):
                    show_image2=show_image2+f[k+1]

                lower_green = np.array([0,90,0], dtype=np.uint8)
                upper_green = np.array([0, 255,0], dtype=np.uint8)
                maskSHI = cv2.inRange(show_image2, lower_green, upper_green)
                show_image2 = cv2.bitwise_and(show_image2,show_image2, mask= maskSHI)

                compare_image = np.zeros((y_img*2,x_img*2,3), np.uint8)
                diff_total = cv2.absdiff(compare_image, show_image2)

                imagen_gris = cv2.cvtColor(diff_total, cv2.COLOR_BGR2GRAY)
                _,contours,_ = cv2.findContours(imagen_gris,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

                for c in contours:
                    if(cv2.contourArea(c) >= 1000):
                        posicion_x,posicion_y,ancho,alto = cv2.boundingRect(c)
                        cv2.rectangle(show_image,(posicion_x,posicion_y),(posicion_x+ancho,posicion_y+alto),(255,0,0),2)

                return positionXarr,positionYarr, show_image

    def run (self):
        self.stop_event.clear()

        while (not self.kill_event.is_set()):
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()
            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)


    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()


    def execute(self):
        # Add your code here
        input_image = self.camera.getImage().data
        global initialTime
        global initTime

        # Show the Filtered_Image on the GUI
        self.setImageFiltered(input_image)

        if input_image is not None:
            hsv = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
            show_image=input_image+1-1
            show_image2=input_image+1-1
            # define range of orange and green color in HSV
            # define range of orange and green color in HSV
            lower_orange = np.array([108,220,69], dtype=np.uint8)                                         # 108,220,69 orange, CAMARA DELANTERA REAL 70,110,120 CAMARA ABAJO DRONE 100,100,80
            upper_orange = np.array([120, 255,110], dtype=np.uint8)                                     #120, 255,110 orange,   CAMARA DELANTERA REAL 120, 200,255 CAMARA ABAJO DRONE 150, 255,255
            maskOrange = cv2.inRange(hsv, lower_orange, upper_orange)
            maskRGBOrange = cv2.bitwise_and(input_image,input_image, mask= maskOrange)
            momentsOrange = cv2.moments(maskOrange)
            areaOrange = momentsOrange['m00']

            lower_green = np.array([20,193,65], dtype=np.uint8) #20,193,65 DELANTERA REAL 10,20,0 ABAJO REAL 0,0,0
            upper_green = np.array([70, 227,100], dtype=np.uint8) #70, 227,100 DELANTERA REAL 40, 200,255 ABAJO REAL 140,100,255
            maskGreen = cv2.inRange(hsv, lower_green, upper_green)
            maskRGBGreen = cv2.bitwise_and(input_image,input_image, mask= maskGreen)
            momentsGreen = cv2.moments(maskGreen)
            areaGreen = momentsGreen['m00']

            kernel = np.ones((3,3),np.uint8)

            maskRGBOrange = cv2.erode(maskRGBOrange,kernel,iterations = 4)
            maskRGBOrange = cv2.dilate(maskRGBOrange,kernel,iterations = 3)

            maskRGBGreen = cv2.erode(maskRGBGreen,kernel,iterations = 4)
            maskRGBGreen = cv2.dilate(maskRGBGreen,kernel,iterations = 3)

            kp=0.01
            kd=0.003

            maskRGBTot = maskRGBOrange+maskRGBGreen
            global x_img
            global y_img
            global yanterior
            global xanterior
            global landed
            global yanteriorTot
            global xanteriorTot
            global numIteracionesOrange
            global numIteracionesGreen

            if(-initialTime+time.time()<10 or initTime==0):
                # If isn't in the air, take off
                if(initTime==0):
                    initialTime=time.time()
                    initTime=1
                    self.extra.takeoff()
                    self.centroImagen(input_image, hsv)
                momentsTot = cv2.moments(maskGreen+maskOrange)
                areaTot = areaGreen + areaOrange
                if momentsTot['m00'] != 0:
                    xTot = int(momentsTot['m10']/momentsTot['m00'])
                    yTot = int(momentsTot['m01']/momentsTot['m00'])

                swi=show_image+1-1
                getImage,f = self.center(show_image,maskRGBOrange,maskRGBGreen)
                positionXarr=[]
                if(len(f)>0):
                    positionXarr,positionYarr,show_image = self.printAndCoord(getImage,swi,f)
                if(len(positionXarr)>0):
                    if(positionXarr[0] != -20 and positionYarr[0]!=-20):
                        vely = (y_img-positionYarr[0])
                        velx = (x_img-positionXarr[0])
                        vytot= vely*kp #0.01
                        vxtot= velx*kp #0.01

                        velxa=1-abs(xanterior-velx)/50 #10
                        if(velxa<0.1):
                            velxa=0.1

                        velya=1-abs(yanterior-vely)/50 #10
                        if(velya<0.1):
                            velya=0.1

                        #self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                        if(y_img-positionYarr[0]<25):
                            vy=0
                        else:
                            vy=vytot*velya*1.4

                        if(x_img-positionXarr[0]<25):
                            vx=0
                        else:
                            vx=vxtot*velxa*1.4

                        self.cmdvel.sendCMDVel(vy*0.01,vx*0.01,0,0,0,0)
                    else:
                        self.cmdvel.sendCMDVel(0,0,0,0,0,0)
                else:
                    self.cmdvel.sendCMDVel(0,0,0,0,0,0)
            elif(-initialTime+time.time()>10 and -initialTime+time.time()<25):
                self.cmdvel.sendCMDVel(0,0,2.5,0,0,0)
                yanterior=0
                xanterior=0
            else:
                global wSearch
                global numVuelta
                global timerW
                global yanterior
                global xanterior
                global var_beacon_status
                if(areaOrange > 0 and areaGreen==0 and numIteracionesOrange<50):
                    numIteracionesOrange=numIteracionesOrange+1

                    xOrange = int(momentsOrange['m10']/momentsOrange['m00'])
                    yOrange = int(momentsOrange['m01']/momentsOrange['m00'])

                    vely = (y_img-yOrange)
                    velx = (x_img-xOrange)

                    vytot= vely*kp
                    vxtot= velx*kp

                    velxa=abs(xanterior-velx)*kd
                    velya=abs(yanterior-vely)*kd

                    vytot=(vytot+velya)
                    vxtot=(vxtot+velxa)

                    if(abs(vxtot-xanteriorTot)>0.3):
                        if(vxtot<xanteriorTot):
                             vxtot = xanteriorTot-0.3
                        else:
                            vxtot = xanteriorTot+0.3

                        if(abs(vytot-yanteriorTot)>0.3):
                             if(vytot<yanteriorTot):
                                 vytot = yanteriorTot-0.3
                             else:
                                 vytot = yanteriorTot+0.3
                             yanteriorTot=vytot
                             xanteriorTot=vxtot
                             self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)
                elif(areaOrange > 0 and areaGreen>0):
                        momentsTot = cv2.moments(maskGreen+maskOrange)
                        areaTot = areaGreen + areaOrange
                        xTot = int(momentsTot['m10']/momentsTot['m00'])
                        yTot = int(momentsTot['m01']/momentsTot['m00'])


                        if((abs(y_img-yTot)<=6 and abs(x_img-xTot)<=6)):
                            global turnland
                            self.extra.land()
                            if(turnland==0):
                                if(areaTot>19272135.0):
                                    turnland=1
                                    landed=time.time()
                                else:
                                    self.cmdvel.sendCMDVel(0,0,-0.5,0,0,0)
                        elif(landed==0):
                            kernel = np.ones((3,3),np.uint8)
                            maskRGBTot = cv2.erode(maskRGBTot,kernel,iterations =2)
                            maskRGBTot = cv2.dilate(maskRGBTot,kernel,iterations =2)
                            vely = (y_img-yTot)
                            velx = (x_img-xTot)
                            vytot= vely*kp
                            vxtot= velx*kp
                            velxa=1-abs(xanterior-velx)/10
                            if(velxa<0.1):
                                velxa=0.1

                            velya=1-abs(yanterior-vely)/10
                            if(velya<0.1):
                                velya=0.1

                            yanterior = y_img-yTot
                            xanterior = x_img-xTot

                            swi=show_image+1-1
                            getImage,f = self.center(show_image,maskRGBOrange,maskRGBGreen)
                            show_image4=getImage
                            positionXarr=[]
                            if(len(f) >0):
                                positionXarr,positionYarr,show_image = self.printAndCoord(getImage,swi,f)

                            blank_image = np.zeros((y_img*2,x_img*2,3), np.uint8)

                            positionX = -20
                            positionY = -20
                            if(len(positionXarr)>0):
                                positionX=positionXarr[0]
                                positionY=positionYarr[0]
                                if(positionX != 0 ):
                                    vely = (y_img-positionYarr[0])
                                    velx = (x_img-positionXarr[0])
                                    vytot= vely*kp
                                    vxtot= velx*kp
                                    velxa=abs(xanterior-velx)*kd
                                    velya=abs(yanterior-vely)*kd

                                    if(abs(vxtot-xanteriorTot)>0.3):
                                       if(vxtot<xanteriorTot):
                                          vxtot = xanteriorTot-0.3
                                       else:
                                          vxtot = xanteriorTot+0.3

                                    if(abs(vytot-yanteriorTot)>0.3):
                                       if(vytot<yanteriorTot):
                                          vytot = yanteriorTot-0.3
                                       else:
                                          vytot = yanteriorTot+0.3
                                    yanterior=velya
                                    xanterior=velxa
                                    self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)
                                    yanteriorTot=vytot
                                    xanteriorTot=vxtot
                                else:
                                    if(abs(vxtot-xanteriorTot)>0.3):
                                       if(vxtot<xanteriorTot):
                                          vxtot = xanteriorTot-0.3
                                       else:
                                          vxtot = xanteriorTot+0.3

                                    if(abs(vytot-yanteriorTot)>0.3):
                                       if(vytot<yanteriorTot):
                                          vytot = yanteriorTot-0.3
                                       else:
                                          vytot = yanteriorTot+0.3
                                    yanterior=velya
                                    xanterior=velxa
                                    self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)
                                    yanteriorTot=vytot
                                    xanteriorTot=vxtot

                            else:
                                velxa=abs(xanterior-velx)*kd
                                velya=abs(yanterior-vely)*kd
                                print(vytot+velya)
                                vytot=(vytot+velya)
                                vxtot=(vxtot+velxa)

                                if(abs(vxtot-xanteriorTot)>0.3):
                                   if(vxtot<xanteriorTot):
                                      vxtot = xanteriorTot-0.3
                                   else:
                                      vxtot = xanteriorTot+0.3

                                if(abs(vytot-yanteriorTot)>0.3):
                                   if(vytot<yanteriorTot):
                                      vytot = yanteriorTot-0.3
                                   else:
                                      vytot = yanteriorTot+0.3

                                yanteriorTot=vytot
                                xanteriorTot=vxtot
                                self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)



                elif(areaOrange == 0 and areaGreen>0):
                    numIteracionesGreen=numIteracionesGreen+1
                    var_beacon_status = 1
                    xGreen = int(momentsGreen['m10']/momentsGreen['m00'])
                    yGreen = int(momentsGreen['m01']/momentsGreen['m00'])
                    if(yanterior==0 and xanterior==0):
                        yanterior = (y_img-yGreen)*0.02
                        xanterior = (x_img-xGreen)*0.02
                        self.cmdvel.sendCMDVel(yanterior,xanterior,0,0,0,0)
                    else:
                        vely = (y_img-yGreen)
                        velx = (x_img-xGreen)
                        vytot= vely*kp
                        vxtot= velx*kp
                        velxa=abs(xanterior-velx)*kd
                        velya=abs(yanterior-vely)*kd
                        vytot=(vytot+velya)
                        vxtot=(vxtot+velxa)

                        if(abs(vxtot-xanteriorTot)>0.3):
                            if(vxtot<xanteriorTot):
                                 vxtot = xanteriorTot-0.3
                            else:
                                vxtot = xanteriorTot+0.3

                            if(abs(vytot-yanteriorTot)>0.3):
                                 if(vytot<yanteriorTot):
                                     vytot = yanteriorTot-0.3
                                 else:
                                     vytot = yanteriorTot+0.3
                                 yanteriorTot=vytot
                                 xanteriorTot=vxtot
                                 self.cmdvel.sendCMDVel(vytot,vxtot,0,0,0,0)



                else:
                    numVuelta=numVuelta+1
                    if(numVuelta>100 and numVuelta < 120):
                        self.cmdvel.sendCMDVel(1.8+wSearch,0,0,0,0,-1.5)
                        timerW=timerW+(timerW/8)
                        if(numVuelta==119):
                            numVuelta=0
                        if(wSearch<1 and numVuelta==101):
                           wSearch=wSearch+0.2
                           numIteracionesGreen=0
                           numIteracionesOrange=0
                    else:
                        self.cmdvel.sendCMDVel(1.8+wSearch,0,0,0,0,1.5 - wSearch)
