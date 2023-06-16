import sys
import math
import rclpy
from rclpy.node import Node
import threading
from math import asin, atan2, pi
from gazebo_msgs.srv import ApplyJointEffort
from std_msgs.msg import String
import time

DEFAULT = 2

class PlatformController(Node):
    def __init__(self, joint):
        '''
        PlatformController Constructor.
        @param joint: Joint name to move
        @type joint: String
        '''
        super().__init__("platform_controller")
        self.lock = threading.Lock()

        # Joint Effort Service
        self.client = self.create_client(ApplyJointEffort, '/apply_joint_effort')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = ApplyJointEffort.Request()
        self.req.joint_name = joint
        
        self.future = None
        self.future_lock = threading.Lock()
    
    def send_effort(self,input_effort):

        self.req.effort = input_effort
        self.req.start_time.sec = 0
        self.req.start_time.nanosec = 0
        self.req.duration.sec = 2000
        self.req.duration.nanosec = 0
        
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        self.future_lock.acquire()
        self.future = future
        #print("self.future: ",self.future.result().success, self.future.result().status_message)
        self.future_lock.release()

class PlatformCommandListener(Node):

    def __init__(self):
        super().__init__("platform_command_listener")
        self.sub = None
        self.applied_effort = 0
        self.controller = PlatformController("lift_joint")
        self.start()

    def start (self):
        '''
        Starts (Subscribes) to the command topic.
        '''
        self.sub = self.create_subscription(String,'/send_effort',self.__callback,10)

    def stop(self):
        '''
        Stops (Unregisters) Node.
        '''
        self.sub.unregister()
    
    def __callback (self, event):
        """
        Load/Unload logic so that effort is not accumulated in joint
        """
        command = event.data
        if (command == "load"):
            if (self.applied_effort == 0):
                effort = DEFAULT
            elif (self.applied_effort < 0):
                effort = self.applied_effort*-2
            else:
                return
        elif (command == "unload"):
            if (self.applied_effort == DEFAULT):
                effort = -DEFAULT
            elif (self.applied_effort > 0):
                effort = self.applied_effort*-2
            else:
                return
        else:
            return
        
        self.controller.send_effort(float(effort))
        self.applied_effort = self.applied_effort + effort