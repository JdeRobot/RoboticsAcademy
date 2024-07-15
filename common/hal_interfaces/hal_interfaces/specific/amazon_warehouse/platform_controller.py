from rclpy.node import Node
from rclpy import spin_until_future_complete
import threading
from gazebo_msgs.srv import ApplyJointEffort
from std_msgs.msg import String

DEFAULT = 2

def cmdLift2String(cmdLift):
    return cmdLift.msg

class CMDLift ():

    def __init__(self):
        self.msg = String()
        self.msg.data = "unload"
    
    def cmd(self,cmd):
        self.msg.data = cmd

    def __str__(self):
        return "CMDlift:" + self.msg.data


### HAL INTERFACE ###
class PublisherPlatformNode(Node):

    def __init__(self, topic):

        super().__init__("PublisherPlatform")
        self.pub = self.create_publisher(String, topic, 10)
        self.data = CMDLift()

    def load(self):
        self.data.cmd("load")
        self.pub.publish(cmdLift2String(self.data))

    def unload(self):
        self.data.cmd("unload")
        self.pub.publish(cmdLift2String(self.data))

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
        spin_until_future_complete(self, future)

        self.future_lock.acquire()
        self.future = future
        #print("self.future: ",self.future.result().success, self.future.result().status_message)
        self.future_lock.release()

class PlatformCommandNode(Node):

    def __init__(self, topic):
        super().__init__("platform_command_listener")
        self.sub = self.create_subscription(
            String, topic, self.listener_callback, 10
        )
        self.applied_effort = 0
        self.controller = PlatformController("lift_joint")

    def listener_callback(self, event):
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
