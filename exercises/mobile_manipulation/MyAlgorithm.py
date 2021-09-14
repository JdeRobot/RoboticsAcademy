import threading
import numpy
from pick_and_place import Pick_Place
from move_base_client import Movebase_Client
from std_msgs.msg import Bool
import os, rospkg, yaml
import rospy


class Algorithm:
    def __init__(self):
        self.is_on = False

        self.stopevent = threading.Event()
        self.pauseevent = threading.Event()

        self.startalgorithm_sub = rospy.Subscriber("/start_algorithm", Bool, self.start_callback)
        self.stopalgorithm_sub = rospy.Subscriber("/stop_algorithm", Bool, self.stop_callback)
        self.pausealgorithm_sub = rospy.Subscriber("/pause_algorithm", Bool, self.pause_callback)
        self.stopalgorithm_pub = rospy.Publisher("/stop_algorithm", Bool, queue_size=0)

    def start_callback(self, msg):
        if msg.data == True:
            self.stopevent.set()
            self.pauseevent.set()
            self.myalgorithm()

            self.pick_place.send_message("Algorithm is finished")

            msg = Bool()
            msg.data = True
            self.stopalgorithm_pub.publish(msg)

    def stop_callback(self, msg):
        if msg.data == True:
            self.pauseevent.clear()
            self.stopevent.clear()

    def pause_callback(self, msg):
        if msg.data == True:
            self.pauseevent.clear()
        else:
            self.pauseevent.set()

    def set_pick_and_place(self, pick_place):
        self.pick_place = pick_place

    def set_client(self, movebase_client):
        self.client = movebase_client

    def move_to(self, target_name):
        ############## Insert your code here ###############
        self.pick_place.send_message("move to "+target_name)
        ####################################################

    def myalgorithm(self):
        ############## Insert your code here ###############
        # Move the robot arm back to home as a start
        self.pick_place.back_to_home()
        
        # insert following two lines where you want to pause or stop the algorithm 
        # with the stop button in GUI
        while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
            if not self.stopevent.isSet():
                return

        ##### A brief example to pick green cylinder on conveyor1 and place it to conveyor2 #####

        ## Move to the target pose to stop in front of conveyor1
        self.move_to("conveyor1")

        ## Pick green cylinder

        ## Move to the target pose to stop in front of conveyor2
        self.move_to("conveyor2")

        ## Place green cylinder
        
        ####################################################


if __name__=="__main__":
    rospy.init_node("pick_place")

    algo = Algorithm()
    algo.set_pick_and_place(Pick_Place())
    algo.set_client(Movebase_Client())
    print("You can start your algorithm with GUI")

    rospy.spin()

# from GUI import GUI
# from HAL import HAL
# # Enter sequential code!

# while True:
#     # Enter iterative code!
#     HAL.back_to_home()
#     target_name = "conveyor1"
#     pose = HAL.get_target_pose(target_name)
#     HAL.send_goal_to_client(pose)
#     while HAL.get_result_from_client() != True:
#         pass
#     print(HAL.get_robot_pose())
#     HAL.move_to_pick_place_home()
#     HAL.spawn_obstacle_rviz("conveyor1")
#     HAL.spawn_all_objects()
#     object_name = "green_cylinder"
#     pose = HAL.get_object_pose(object_name)
#     pose.position.z -= 0.01
#     HAL.pickup(object_name, pose.position, 0.48)
#     HAL.back_to_home(False)
#     target_name = "conveyor2"
#     pose = HAL.get_target_pose(target_name)
#     HAL.send_goal_to_client(pose)
#     while HAL.get_result_from_client() != True:
#         pass
#     print(HAL.get_robot_pose())
#     HAL.move_to_pick_place_home(False)
#     HAL.spawn_obstacle_rviz(target_name)
#     pose = HAL.get_target_position(target_name)
#     HAL.place(object_name, pose)