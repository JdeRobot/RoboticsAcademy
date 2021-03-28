import threading
import rospy

from drone_wrapper import DroneWrapper
from interfaces.autonomous_mouse import path_obfuscated
from interfaces.threadStoppable import StoppableThread
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState


class Mouse:
    def __init__(self):
        self.mouse = DroneWrapper(name="rqt", ns="mouse/")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def takeoff(self):
        self.mouse.takeoff(h=5)

    def start_mouse(self, path_level):
        self.takeoff()
        self.thread = StoppableThread(target=self.autonomous_mouse, args=[path_level])
        self.thread.start()

    def land(self):
        self.mouse.land()

    def stop_mouse(self):
        self.thread.stop()
        self.mouse.set_cmd_mix(z=self.mouse.get_position()[2])

    def reset_mouse(self):
        self.thread.stop()
        self.mouse.land()

        reset_state = rospy.ServiceProxy('/gazebo/set_model_state', GetModelState)
        req = ModelState()
        req.model_name = "iris_red"
        req.pose.position = [0.0, 0.0, 0.05]
        req.pose.orientation = [0.0, 2.0, 0.0, 1.0]
        req.twist.linear = [0.0, 0.0, 0.0]
        req.twist.angular = [0.0, 0.0, 0.0]
        reset_state(req)

    def autonomous_mouse(self, path_level):
        while True:
            v = path_obfuscated(path_level)
            if v is not None:
                vx, vy, vz, yaw = v
                self.mouse.set_cmd_vel(vx, vy, vz, yaw)
            else:
                print("[Mouse] Path {} not available".format(path_level))
                break
