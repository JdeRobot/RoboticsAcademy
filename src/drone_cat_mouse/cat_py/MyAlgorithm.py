import threading
import time
from datetime import datetime
import signal
import sys
import numpy as np
import cv2
from math import atan2, cos, pi

import cv2
import numpy as np

from parallelIce.cameraClient import CameraClient
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient

time_cycle = 80

area_height = [[50.0,4.47],[52.0,4.46],[53.0,4.44],[54.5,4.36],[56.0,4.34],[56.5,4.3],[58.5,4.29],[59.0,4.23],[59.5,4.22],[60.5,4.21],[61.0,4.2],[62.0,4.17],[62.5,4.16],[65.0,4.11],[65.5,4.1],[66.0,4.09],[66.5,4.08],[68.5,4.07],[69.5,4.06],[70.5,4.04],[71.0,4.02],[72.0,3.99],[74.0,3.98],[75.0,3.96],[78.0,3.95],[78.5,3.94],[79.0,3.92],[79.5,3.9],[80.5,3.85],[81.5,3.83],[82.0,3.82],[82.5,3.8],[83.0,3.79],[83.5,3.78],[85.5,3.77],[86.0,3.76],[86.5,3.75],[87.5,3.74],[89.0,3.73],[89.5,3.72],[90.0,3.71],[91.0,3.69],[93.5,3.68],[95.5,3.66],[96.5,3.65],[98.5,3.64],[99.5,3.63],[100.0,3.62],[101.0,3.55],[101.5,3.54],[102.5,3.52],[103.5,3.5],[104.0,3.49],[104.5,3.48],[105.5,3.47],[109.0,3.46],[110.0,3.45],[112.5,3.44],[114.0,3.42],[119.0,3.41],[122.5,3.4],[124.0,3.38],[124.5,3.37],[125.0,3.35],[125.5,3.33],[126.0,3.27],[126.5,3.26],[127.5,3.25],[130.5,3.24],[134.5,3.23],[136.5,3.22],[140.0,3.21],[143.5,3.2],[147.0,3.19],[148.5,3.15],[149.0,3.13],[149.5,3.11],[150.0,3.1],[151.0,3.09],[152.0,3.07],[156.5,3.06],[158.5,3.05],[159.0,3.04],[161.5,3.03],[167.5,3.02],[173.0,3.01],[178.0,2.98],[178.5,2.97],[179.0,2.96],[180.0,2.94],[180.5,2.92],[181.0,2.91],[182.0,2.9],[185.0,2.88],[189.5,2.87],[194.5,2.86],[204.0,2.85],[210.0,2.84],[211.0,2.83],[215.0,2.82],[220.0,2.81],[221.5,2.8],[225.0,2.79],[233.0,2.78],[242.5,2.77],[248.5,2.76],[392.5,2.75],[435.5,2.74],[438.5,2.73],[447.0,2.72],[462.5,2.71],[693.0,2.7],[706.0,2.69],[708.5,2.68],[753.5,2.67],[756.5,2.66],[807.5,2.65],[997.0,2.64],[1031.0,2.63],[1039.5,2.62],[1048.5,2.61],[1054.0,2.6],[1070.5,2.59],[1088.0,2.58],[1088.5,2.57],[1113.0,2.56],[1165.5,2.55],[1170.5,2.54],[1292.5,2.53],[1398.5,2.52],[1411.0,2.51],[1420.5,2.5],[1436.5,2.49],[1468.5,2.48],[1482.5,2.47],[1498.5,2.46],[1517.5,2.45],[1525.5,2.44],[1537.5,2.43],[1556.5,2.42],[1566.5,2.41],[1582.0,2.4],[1598.0,2.39],[1616.5,2.38],[1625.5,2.37],[1646.5,2.36],[1665.0,2.35],[1678.5,2.34],[1694.5,2.33],[1708.5,2.32],[1725.0,2.31],[1740.5,2.3],[1754.0,2.29],[1777.5,2.28],[1797.5,2.27],[1818.0,2.26],[1834.5,2.25],[1857.0,2.24],[1871.5,2.23],[1890.5,2.22],[1906.0,2.21],[1924.0,2.2],[1945.0,2.19],[1974.5,2.18],[1986.5,2.17],[2013.0,2.16],[2032.0,2.15],[2055.0,2.14],[2075.5,2.13],[2091.0,2.12],[2117.0,2.11],[2148.0,2.1],[2171.0,2.09],[2187.5,2.08],[2214.0,2.07],[2239.5,2.06],[2266.0,2.05],[2281.5,2.04],[2309.5,2.03],[2347.5,2.02],[2371.0,2.01],[2390.0,2.0],[2415.0,1.99],[2445.5,1.98],[2476.0,1.97],[2501.0,1.96],[2537.0,1.95],[2565.5,1.94],[2589.5,1.93],[2624.5,1.92],[2654.5,1.91],[2687.5,1.9],[2718.0,1.89],[2763.0,1.88],[2796.5,1.87],[2826.0,1.86],[2854.5,1.85],[2896.0,1.84],[2923.0,1.83],[2968.5,1.82],[3010.5,1.81],[3037.5,1.8],[3081.5,1.79],[3119.5,1.78],[3168.5,1.77],[3205.5,1.76],[3248.5,1.75],[3290.5,1.74],[3344.5,1.73],[3387.5,1.72],[3420.5,1.71],[3456.5,1.7],[3523.0,1.69],[3573.0,1.68],[3635.0,1.67],[3661.5,1.66],[3700.0,1.65],[3756.0,1.64],[3827.5,1.63],[3890.5,1.62],[3926.5,1.61],[3968.5,1.6],[4065.0,1.59],[4123.0,1.58],[4163.5,1.57],[4227.5,1.56],[4321.5,1.55],[4371.0,1.54],[4421.5,1.53],[4490.0,1.52],[4556.5,1.51],[4645.5,1.5],[4689.0,1.49],[4773.5,1.48],[4852.5,1.47],[4931.5,1.46],[5009.0,1.45],[5118.5,1.44],[5181.5,1.43],[5253.0,1.42],[5377.5,1.41],[5456.0,1.4],[5503.5,1.39],[5652.0,1.38],[5731.5,1.37],[5834.0,1.36],[5940.5,1.35],[6048.5,1.34],[6169.0,1.33],[6287.0,1.32],[6355.5,1.31],[6477.5,1.3],[6607.5,1.29],[6761.0,1.28],[6838.5,1.27],[6994.5,1.26],[7089.0,1.25],[7259.5,1.24],[7351.5,1.23],[7477.0,1.22],[7682.0,1.21],[7817.5,1.2],[7943.0,1.19],[8091.0,1.18],[8398.5,1.17],[8541.5,1.16],[8715.0,1.15],[8917.0,1.14],[9341.5,1.12],[9809.5,1.1],[10048.5,1.08],[10645.5,1.06],[11042.0,1.04],[11432.0,1.03],[11943.0,1.01],[12471.5,0.99],[12977.0,0.97],[13815.0,0.95],[14725.0,0.94],[15603.5,0.91],[16743.0,0.88],[18015.5,0.86],[19582.5,0.83],[21510.0,0.81],[23552.0,0.77],[25895.5,0.74],[28986.0,0.72],[31793.0,0.69],[34777.0,0.67],[37782.0,0.65],[41082.5,0.63],[43899.0,0.62],[48169.5,0.6],[50271.0,0.59]]

def nothing(x):
	pass

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.cmdvel.sendCMDVel(0, 0, 0, 0, 0, 0)
    	self.extra.takeoff()

        self.image = None

    	self.diff_time = 0
    	self.search_inc = 1
    	self.rotate_inc = 2
    	self.rotate_inc2 = 2
    	self.height = 240
    	self.width = 320
    	self.camera2 = 0
    	self.reset_pid_v()

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def setImageFiltered(self, image):
        self.lock.acquire()
        self.image=image
        self.lock.release()

    def getImageFiltered(self):
        self.lock.acquire()
        tempImage=self.image
        self.lock.release()
        return tempImage

    def reset_pid_v(self):
    	self.KPX = 0.006
    	self.KIX = 0.0250
    	self.KDX = 0.0020
    	self.KPY = 0.008
    	self.KIY = 0.0250
    	self.KDY = 0.0020
    	self.KPZ = 1
    	self.KIZ = 0.2
    	self.KDZ = 0.3
    	self.KPR = 1
    	self.KIR = 0.2
    	self.KDR = 0.3
    	self.Z_SETPOINT = 2.5
    	self.X_SETPOINT = self.width / 2
    	self.Y_SETPOINT = self.height / 2
    	self.common_pid(1)

    def reset_pid_f(self):

    	self.KPX = 0.036
    	self.KIX = 0.0250
    	self.KDX = 0.0020
    	self.KPY = 0.016
    	self.KIY = 0.0
    	self.KDY = 0.0020
    	self.KPZ = 0.001
    	self.KIZ = 0.2
    	self.KDZ = 0.3
    	self.Z_SETPOINT = 500
    	self.X_SETPOINT = self.width / 2
    	self.Y_SETPOINT = self.height / 2
    	self.common_pid(5)

    def common_pid(self, z_range):
    	self.previous_time = time.time()
    	self.previous_x_error = 0.0
    	self.previous_y_error = 0.0
    	self.previous_z_error = 0.0
    	self.previous_r_error = 0.0
    	self.x_integral = 0.0
    	self.y_integral = 0.0
    	self.z_integral = 0.0
    	self.r_integral = 0.0
    	self.z_average = []
    	for index in range(z_range):
    		self.z_average.append(self.Z_SETPOINT)
    	self.index_z_average = 0

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
        current_time = time.time()
    	dt = current_time - self.previous_time
    	self.previous_time = current_time

    	droneImage = self.camera.getImage().data
    	hsv = cv2.cvtColor(droneImage, cv2.COLOR_BGR2HSV)
    	#lower_green = np.array([57,255,0])
    	#upper_green = np.array([61,255,255])
    	lower_green = np.array([10,40,10])
    	upper_green = np.array([255,42,46])

    	mask = cv2.inRange(droneImage, lower_green, upper_green)
        image_filtered = np.dstack((mask, mask, mask))
        self.setImageFiltered(image_filtered)

    	im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    	print("contours:", contours)

        # Buscar el contorno mas grande

        maximum_area = 0
        bigger_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > maximum_area:
                maximum_area = area
                bigger_contour = contour


            # Encontrar el centroide  del contorno elegido
    	M = None
        if bigger_contour is not None:
    		M = cv2.moments(bigger_contour)
    		centroid_x = int(M['m10']/M['m00'])
    		centroid_y = int(M['m01']/M['m00'])
    		cv2.circle(droneImage, (centroid_x, centroid_y), 5, 255, -1)

            # Indicar las instrucciones dependiendo de si encuentra el contorno
    	if M is not None:
    		text = str(M['m00'])
    		# encontrado con camara ventral
    		if self.camera2 == 0:

    			pid_x = self.calculate_pid(self.X_SETPOINT, centroid_x, self.x_integral, self.previous_x_error, dt, self.KPX, self.KIX, self.KDX)
    			pid_y = self.calculate_pid(self.Y_SETPOINT, centroid_y, self.y_integral, self.previous_y_error, dt, self.KPY, self.KIY, self.KDY)

    			calculated = self.area_convert(maximum_area, area_height, 5.0)
    			if (centroid_x > 30 and centroid_x < (self.width - 30)) and (centroid_y > 30 and centroid_y < (self.height - 30)):
    				pid_z = self.calculate_pid(self.Z_SETPOINT, calculated, self.z_integral, self.previous_z_error, dt, self.KPZ, self.KIZ, self.KDZ)
    			else:
    				pid_z = 0

    			pid_r = self.calculate_pid(self.get_angle(self.X_SETPOINT, self.Y_SETPOINT, centroid_x, centroid_y), 0, self.r_integral, self.previous_r_error, dt, self.KPR, self.KIR, self.KDR)
    			#self.rotate_inc = 0
    			if self.get_angle(self.X_SETPOINT, self.Y_SETPOINT, centroid_x, centroid_y) < 0:
    				self.rotate_inc = -1
    			else:
    				self.rotate_inc = 1
    			self.search_inc = -2
    			self.diff_time = current_time
    		# encontrado con camara delantera
    		else:
    			calculated = maximum_area
    			pid_x = 0
    			if calculated < 550:
    				pid_y = max((550 - calculated) * 0.004, 0)
    				pid_z = self.calculate_pid(self.Y_SETPOINT, centroid_y, self.y_integral, self.previous_y_error, dt, self.KPY, self.KIY, self.KDY)
    			else:
    				pid_y = 0
    				pid_z = 0.5

    			if pid_y > 0:
    				if centroid_y < 20:
    					self.search_inc = -2
    				elif centroid_y > (self.height - 20):
    					self.search_inc = 2

    			pid_r = self.calculate_pid(self.X_SETPOINT, centroid_x, self.x_integral, self.previous_x_error, dt, self.KPX, self.KIX, self.KDX)
    			self.rotate_inc = 0
    			if pid_r < 0:
    				self.rotate_inc2 = -1
    			else:
    				self.rotate_inc2 = 1

    			self.diff_time = current_time
    	#buscar el raton
    	else:
    		if self.camera2 == 0:
    			self.reset_pid_f()
    			self.extra.toggleCam()
    			self.camera2 = 1
    		else:
    			self.reset_pid_v()
    			self.extra.toggleCam()
    			self.camera2 = 0
    		if self.rotate_inc == 0 and (current_time - self.diff_time) > 1:
    			 self.rotate_inc = self.rotate_inc2
    		pid_r = self.rotate_inc
    		pid_x = 3
    		pid_y = 3

    		height = self.pose.getPose3d().z
    		if height < 2:
    			self.search_inc = 2
    		elif height > 6:
    			self.search_inc = -2
    		pid_z = self.search_inc
    		calculated = 0

    		text = 'X'

    	self.cmdvel.sendCMDVel(pid_x, pid_y, pid_z, 0, 0, pid_r)

    def calculate_pid(self, setpoint, situation, integral, previous, dt, kp, ki, kd):
    	error = setpoint - situation
    	integral += error * dt
    	pid = error * kp + integral * ki + (error - previous) * kd
    	previous = error
    	return pid

    def area_convert(self, area, array, maximum):
    	previous = [0, maximum]
    	for data in array:
    		if data[0] < area:
    			previous = data
    		else:
    			return previous[1]
    	return 0

    def insert_height(self, value):
    	self.z_average[self.index_z_average] = value
    	self.index_z_average += 1
    	if self.index_z_average == len(self.z_average):
    		self.index_z_average = 0
    	sum = 0
    	for item in self.z_average:
    		sum += item
    	return sum / len(self.z_average)


    def get_angle(self, x1, y1, x2, y2):
    	dx = x2 - x1
    	dy = y2 - y1
    	return -cos(atan2(dy,dx))
