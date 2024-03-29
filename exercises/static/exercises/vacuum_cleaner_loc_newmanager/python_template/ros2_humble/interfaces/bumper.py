import sys
import math
import rclpy
from rclpy.node import Node
import threading
from math import asin, atan2, pi
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import GetEntityState


class BumperData ():

	def __init__(self):

		self.bumper = 0 # Indicates that bumper is
		self.state = 0 # pressed or no (1,0)
		self.timeStamp = 0 # Time stamp [s]


	def __str__(self):
		s = "BumperData: {\n   bumper: " + str(self.bumper) + "\n   state: " + str(self.state)
		s = s + "\n   timeStamp: " + str(self.timeStamp)  + "\n}"
		return s 

class ListenerBumper(Node):
    '''
        ROS Bumper Subscriber. Bumper Client to Receive Contacts State from ROS nodes.
    '''
    def __init__(self, topic, robot):
        '''
        ListenerBumper Constructor.

        @param topic: ROS topic to subscribe
        
        @type topic: String

        '''
        super().__init__("bumper_subscriber_node")
        self.topic = topic
        self.data = BumperData()
        self.sub = None
        self.lock = threading.Lock()
        self.start()

        self.client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetEntityState.Request()
        self.req.name = robot
        self.future = None
        self.future_lock = threading.Lock()

    def spin_bumper_node(self):
        
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        self.future_lock.acquire()
        self.future = future
        self.future_lock.release()
 
    def __callback (self, event):
        '''
        Callback function to receive and save Contacts State. 

        @param event: ROS ContactsState received
        
        @type event: ContactsState

        '''
        bump = self.contactsState2BumperData(event)

        self.lock.acquire()
        self.data = bump
        self.lock.release()


    def stop(self):
        '''
        Stops (Unregisters) the client.

        '''
        self.sub.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client.

        '''
        self.sub = self.create_subscription(ContactsState, self.topic , self.__callback,10)

    def getBumperData(self):
        '''
        Returns last BumperData. 

        @return last JdeRobotTypes BumperData saved

        '''
        self.lock.acquire()
        bump = self.data
        self.lock.release()
        
        return bump
    
    def contactsState2BumperData(self,event):
        '''
        Translates from ROS ContactsState to JderobotTypes BumperData. 

        @param event: ROS ContactsState to translate

        @type event: ContactsState

        @return a BumperData translated from event

        # bumper
        LEFT   = 0
        CENTER = 1
        RIGHT  = 2

        #  state
        RELEASED = 0
        PRESSED  = 1

        '''
        bump = BumperData()
        if len(event.states) > 0:
            contact_x = event.states[0].contact_normals[0].x
            contact_y = event.states[0].contact_normals[0].y

            self.future_lock.acquire()
            robot_coordinates = self.future.result()
            self.future_lock.release()

            if robot_coordinates is not None:

                qx = robot_coordinates.state.pose.orientation.x
                qy = robot_coordinates.state.pose.orientation.y
                qz = robot_coordinates.state.pose.orientation.z
                qw = robot_coordinates.state.pose.orientation.w

                rotateZa0=2.0*(qx*qy + qw*qz)
                rotateZa1=qw*qw + qx*qx - qy*qy - qz*qz
                robotYaw=0.0
                if(rotateZa0 != 0.0 and rotateZa1 != 0.0):
                    robotYaw=math.atan2(rotateZa0,rotateZa1)

                global_contact_angle = 0.0
                if(contact_x != 0.0 and contact_y != 0.0):
                    global_contact_angle = math.atan2(contact_y,contact_x)

                relative_contact_angle = global_contact_angle - robotYaw -math.pi

                bump.state = 1
                if relative_contact_angle <= math.pi/2 and relative_contact_angle > math.pi/6: #0.52, 1.5
                    bump.bumper = 0
                elif relative_contact_angle <= math.pi/6 and relative_contact_angle > -math.pi/6: # -0.52, 0.52
                    bump.bumper = 1
                elif relative_contact_angle < -math.pi/6 and relative_contact_angle >= -math.pi/2: #-1.5 , -0.52
                    bump.bumper = 2
            else:
                bump.state = 0
        else:
            bump.state = 0

        bump.timeStamp = event.header.stamp.sec + (event.header.stamp.nanosec *1e-9)
        return bump