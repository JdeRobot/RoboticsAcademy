import sys
import rospy
from threading import Event

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from interfaces.threadStoppable import StoppableThread

class Turtlebot():
    def __init__(self):
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.play_event = Event()
        rospy.sleep(2)
        self.stop_turtlebot()

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def start_turtlebot(self):
        try:
            self.play_event.set()
            rospy.sleep(0.5)
            self.thread.join()
        except:
            print("Thread not yet started")

    def stop_turtlebot(self):
        self.play_event.clear()
        self.thread = StoppableThread(target=self.__stop__, args=[])
        self.thread.start()

    def __stop__(self):
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
            if self.play_event.is_set():
                sys.exit() # kill stop_turtlebot thread
            else:
                self.set_state(req)

    def reset_turtlebot(self):
        self.start_turtlebot()
        req = ModelState()
        req.model_name = "turtlebot3"
        req.twist.linear.x = 0.0
        req.twist.linear.y = 0.0
        req.twist.linear.z = 0.0
        req.twist.angular.x = 0.0
        req.twist.angular.y = 0.0
        req.twist.angular.z = 0.0
        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = 0.0
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = 0.0
        req.pose.orientation.w = 0.0
        self.set_state(req)
        self.stop_turtlebot()








