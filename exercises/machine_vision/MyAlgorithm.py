# import threading
# import numpy
# from pick_and_place import Pick_Place
# from std_msgs.msg import Bool
# import os, rospkg, yaml
# import rospy


# class Algorithm:
#     def __init__(self):
#         self.is_on = False

#         self.stopevent = threading.Event()
#         self.pauseevent = threading.Event()

#         self.startalgorithm_sub = rospy.Subscriber("/start_algorithm", Bool, self.start_callback)
#         self.stopalgorithm_sub = rospy.Subscriber("/stop_algorithm", Bool, self.stop_callback)
#         self.pausealgorithm_sub = rospy.Subscriber("/pause_algorithm", Bool, self.pause_callback)
#         self.stopalgorithm_pub = rospy.Publisher("/stop_algorithm", Bool, queue_size=0)

#     def start_callback(self, msg):
#         if msg.data == True:
#             self.stopevent.set()
#             self.pauseevent.set()
#             self.myalgorithm()

#             self.pick_place.send_message("Algorithm is finished")

#             msg = Bool()
#             msg.data = True
#             self.stopalgorithm_pub.publish(msg)        

#     def stop_callback(self, msg):
#         if msg.data == True:
#             self.pauseevent.clear()
#             self.stopevent.clear()

#     def pause_callback(self, msg):
#         if msg.data == True:
#             self.pauseevent.clear()
#         else:
#             self.pauseevent.set()

#     def set_pick_and_place(self, pick_place):
#         self.pick_place = pick_place

#     def build_map(self):
#         ############## Insert your code here ###############
#         self.pick_place.send_message("Building map")

#         ####################################################

#     def get_object_position(self, object_name):
#         ############## Insert your code here ###############

#         return position
#         ####################################################

#     def myalgorithm(self):
#         ############## Insert your code here ###############
#         self.build_map()

#         # Move the robot back to home as a start
#         self.pick_place.back_to_home()
        
#         # insert following two lines where you want to pause or stop the algorithm 
#         # with the stop button in GUI
#         while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
#             if not self.stopevent.isSet():
#                 return

#         ##### A brief example to pick and place object "green_cylinder" #####

#         # get object position and pick it up
#         # parameters HEIGHT_OFFSET need to be tuned according to the object height
#         object_name = "green_cylinder"

#         position = self.get_object_position(object_nam)
#         self.pick_place.pickup(object_name, position)

#         # setup stop signal detector
#         while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
#             if not self.stopevent.isSet():
#                 return

#         # choose target position and place the object
#         target_name = "target6"
#         position = self.pick_place.get_target_position(target_name)
#         self.pick_place.place(object_name, position)

#         ####################################################


# if __name__=="__main__":
#     rospy.init_node("pick_place_basic")

#     algo = Algorithm()
#     algo.set_pick_and_place(Pick_Place())
#     print("You can start your algorithm with GUI")

#     rospy.spin()
import threading
import numpy
from pick_and_place import Pick_Place
from std_msgs.msg import Bool
import os, rospkg, yaml
import rospy
import time

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

    def build_map(self):
        ############## Insert your code here ###############
        self.pick_place.send_message("Building map")
        ####################################################

    def get_object_position(self, object_name):
        ############## Insert your code here ###############
        color = "green"
        shape = "cylinder"
        self.pick_place.start_color_filter(color, 50, 0, 255, 100, 50, 0)
        self.pick_place.start_shape_filter(color, shape, 0.05)

        rospy.sleep(3)
        
        position = self.pick_place.get_object_position(object_name)

        return position
        ####################################################

    def myalgorithm(self):
        ############## Insert your code here ###############
        self.build_map()

        # Move the robot back to home as a start
        self.pick_place.back_to_home()
        
        # insert following two lines where you want to pause or stop the algorithm 
        # with the stop button in GUI
        while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
            if not self.stopevent.isSet():
                return

        ##### A brief example to pick and place object "green_cylinder" #####

        # get object position and pick it up
        # parameters HEIGHT_OFFSET need to be tuned according to the object height
        object_name = "green_cylinder"

        # self.get_object_position(object_name)
        position = self.get_object_position(object_name)
        self.pick_place.pickup(object_name, position)

        # # setup stop signal detector
        # while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
        #     if not self.stopevent.isSet():
        #         return

        # # choose target position and place the object
        # target_name = "target6"
        # position = self.pick_place.get_target_position(target_name)
        # self.pick_place.place(object_name, position)

        while (not self.pauseevent.isSet()) or (not self.stopevent.isSet()):
            if not self.stopevent.isSet():
                return

        ####################################################

if __name__=="__main__":
    rospy.init_node("pick_place_basic")
    algo = Algorithm()
    algo.set_pick_and_place(Pick_Place())
    print("You can start your algorithm with GUI")

    rospy.spin()