import rospy
from math import sqrt
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from threading import Event, Thread


MIN_DIST = 2.0 # minimum distance required by drone to pick package


class Magnet:
    def __init__(self):
        # rospy.init_node("magnet")
        self.magnetize = Event()
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


    def start_magnet(self):
        req = ModelState()
        req.model_name = "package_box"

        while self.magnetize.is_set():
            drone_pos = self.get_state("typhoon_h480_dual_cam", "")
            req.pose.position.x = drone_pos.pose.position.x
            req.pose.position.y = drone_pos.pose.position.y
            req.pose.position.z = drone_pos.pose.position.z - 0.215
            req.twist.linear.x = drone_pos.twist.linear.x
            req.twist.linear.y = drone_pos.twist.linear.y
            req.twist.linear.z = drone_pos.twist.linear.z
            req.twist.angular.x = drone_pos.twist.angular.x
            req.twist.angular.y = drone_pos.twist.angular.y
            req.twist.angular.z = drone_pos.twist.angular.z
            req.pose.orientation.x = drone_pos.pose.orientation.x
            req.pose.orientation.y = drone_pos.pose.orientation.y
            req.pose.orientation.z = drone_pos.pose.orientation.z
            req.pose.orientation.w = drone_pos.pose.orientation.w
            self.set_state(req)


    def stop_magnet(self):
        self.magnetize.clear()


    def pick_pkg(self):
        try:
            pkg_pos = self.get_state("package_box", "")
            drone_pos = self.get_state("typhoon_h480_dual_cam", "")
            dist = sqrt(pow(pkg_pos.pose.position.x - drone_pos.pose.position.x, 2) + \
                        pow(pkg_pos.pose.position.y - drone_pos.pose.position.y, 2) + \
                        pow(pkg_pos.pose.position.z - drone_pos.pose.position.z, 2))

            if dist < MIN_DIST:
                self.magnetize.set()
                self.pick_pkg_thread = Thread(target=self.start_magnet)
                self.pick_pkg_thread.start()
        except:
            pass


    def drop_pkg(self):
        try:
            self.stop_magnet()
            self.pick_pkg_thread.join()
        except:
            pass
