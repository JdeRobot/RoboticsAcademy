import threading
import numpy
# from interfaces.robot_wrapper import RobotWrapper
from pick_and_place import Pick_Place
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

    def myalgorithm(self):
        ############## Insert your code here ###############
        # Move the robot back to home as a start
        self.pick_place.back_to_home()
        
        # insert following two lines where you want to pause or stop the algorithm 
        # with the stop button in GUI
        while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
            if not self.stopevent.isSet():
                return

        ##### A brief example to pick and place object "blue_ball" #####
        object_name = "blue_ball"
        # get object pose
        pose = self.pick_place.get_object_pose(object_name)

        # generate grasp message and pick it up
        # parameters WIDTH and LENGTH need to be tuned according to the object and grasping pose
        WIDTH = 0.3
        LENGTH = 0.15
        grasp = self.pick_place.generate_grasp(object_name, "vertical", pose.position, WIDTH, length=LENGTH)
        self.pick_place.pickup(object_name, [grasp])

        # setup stop signal detector
        while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
            if not self.stopevent.isSet():
                return

        # choose target position and place the object
        target_name = "blue_target"
        place_position = self.pick_place.get_target_position(target_name)
        self.pick_place.place("vertical", place_position)

        ####################################################



if __name__=="__main__":
    rospy.init_node("pick_place")

    algo = Algorithm()
    algo.set_pick_and_place(Pick_Place())
    print("You can start your algorithm with GUI")

    rospy.spin()