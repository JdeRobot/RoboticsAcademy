import threading
from threading import Event
import rospy
import sys

from interfaces.threadStoppable import StoppableThread
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class Turtlebot():
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.play = Event()

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def start_turtlebot(self):
        try:
            self.play.set() # play=True
            # wait for stop_turtlebot_thread to close
            rospy.sleep(0.5)
            self.thread.join()
        except:
            print("Thread not yet started")

    def stop_turtlebot(self):
        self.play.clear() # play=False
        self.thread = StoppableThread(name="stop_turtlebot_thread", target=self.halt_turtlebot, args=[])
        self.thread.start()

    def halt_turtlebot(self):
        rec = self.get_state("turtlebot3", "base_link")
        req = ModelState()
        req.model_name = "turtlebot3"
        req.twist.linear.x = 0.0
        req.twist.linear.y = 0.0
        req.twist.linear.z = 0.0
        req.twist.angular.x = 0.0
        req.twist.angular.y = 0.0
        req.twist.angular.z = 0.0
        req.pose.position.x = rec.pose.position.x + 3.0
        req.pose.position.y = rec.pose.position.y + 1.0
        req.pose.position.z = 0.0
        req.pose.orientation.x = rec.pose.orientation.x
        req.pose.orientation.y = rec.pose.orientation.y
        req.pose.orientation.z = rec.pose.orientation.z
        req.pose.orientation.w = 1.0
        while True:
            if self.play.is_set():
                sys.exit() # kill stop_turtlebot_thread if play==True
            else:
                self.set_state(req)

    def reset_turtlebot(self):
        self.start_turtlebot() # start turtlebot first

        req = ModelState()
        req.model_name = "turtlebot3"
        req.twist.linear.x = 0.0
        req.twist.linear.y = 0.0
        req.twist.linear.z = 0.0
        req.twist.angular.x = 0.0
        req.twist.angular.y = 0.0
        req.twist.angular.z = 0.0
        req.pose.position.x = 0.0
        req.pose.position.y = 3.0
        req.pose.position.z = 0.0
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = 0.0
        req.pose.orientation.w = 0.0
        self.set_state(req)








