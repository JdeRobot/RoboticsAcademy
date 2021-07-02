import rospy
import sys
from threading import Event

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from interfaces.threadStoppable import StoppableThread

class Car():
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.play_event = Event()
        self.curr_posx = 5.0 #initial position
        self.base_speed = 0.0001
        self.level0 = self.base_speed * 1
        self.level1 = self.base_speed * 10
        self.level2 = self.base_speed * 20
        self.level3 = self.base_speed * 30

    #Explicit initialization functions
    #Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def set_pos_car(self, x=0):
        req = ModelState()
        req.model_name = "car_color_beacon"
        req.pose.position.x = x
        self.set_state(req)

    def __start__(self, path_level):
        while self.play_event.is_set():
            rospy.sleep(0.01)
            if path_level == 0:
                self.set_pos_car(self.curr_posx)
                self.curr_posx = self.curr_posx + self.level0
            elif path_level == 1:
                self.set_pos_car(self.curr_posx)
                self.curr_posx = self.curr_posx + self.level1
            elif path_level == 2:
                self.set_pos_car(self.curr_posx)
                self.curr_posx = self.curr_posx + self.level2
            elif path_level == 3:
                self.set_pos_car(self.curr_posx)
                self.curr_posx = self.curr_posx + self.level3
        else:
            sys.exit() #exit thread

    def start_car(self, path_level):
        self.play_event.set()
        self.thread = StoppableThread(target=self.__start__, args=[path_level,])
        self.thread.start()

    def stop_car(self):
        try:
            self.play_event.clear()
            rospy.sleep(0.5)
            self.thread.join()
        except:
            pass

    def reset_car(self):
        try:
            self.play_event.clear()
            rospy.sleep(0.5)
            self.thread.join()
        except:
            pass
        self.curr_posx = 5.0
        self.set_pos_car(self.curr_posx) #set initial position