import threading


class Sensors:

    def __init__ (self, motors, pose3d, laser):
        ''' Sensors class collects the data from the sensors at every moment. '''
        self.motors = motors
        self.pose3d = pose3d
        self.laser = laser
        self.lock = threading.Lock()

    def getPose3D(self):
        return self.pose3d

    def getLaser(self):
        return self.laser

    def getMotors(self):
        return self.motors

    def update(self):
        ''' Updates the sensors observations every time the thread changes. '''
    
        self.lock.acquire()

        self.laserdata = self.laser.getLaserData()
        self.actualPose = self.pose3d.getPose3d()

        self.lock.release()
