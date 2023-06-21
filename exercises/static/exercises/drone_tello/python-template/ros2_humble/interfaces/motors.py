import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction
from tello_msgs.msg import TelloResponse
from std_msgs.msg import String
import threading
from math import pi as PI
from .threadPublisher import ThreadPublisher
import os

file_path = os.path.join(os.path.sep, "log_pub_motors.txt")


def cmdvel2Twist(vel):

    tw = Twist()
    tw.linear.x = float(vel.vx)
    tw.linear.y = float(vel.vy)
    tw.linear.z = float(vel.vz)
    tw.angular.x = float(vel.ax)
    tw.angular.y = float(vel.ay)
    tw.angular.z = float(vel.az)

    return tw


class CMDVel ():

    def __init__(self):

        self.vx = 0 # vel in x[m/s] (use this for V in wheeled robots)
        self.vy = 0 # vel in y[m/s]
        self.vz = 0 # vel in z[m/s]
        self.ax = 0 # angular vel in X axis [rad/s]
        self.ay = 0 # angular vel in X axis [rad/s]
        self.az = 0 # angular vel in Z axis [rad/s] (use this for W in wheeled robots)
        self.timeStamp = 0 # Time stamp [s]


    def __str__(self):
        s = "CMDVel: {\n   vx: " + str(self.vx) + "\n   vy: " + str(self.vy)
        s = s + "\n   vz: " + str(self.vz) + "\n   ax: " + str(self.ax) 
        s = s + "\n   ay: " + str(self.ay) + "\n   az: " + str(self.az)
        s = s + "\n   timeStamp: " + str(self.timeStamp)  + "\n}"
        return s 

class PublisherMotors(Node):
 
    def __init__(self, topic, maxV, maxW):
        super().__init__("DroneTello")
        self.maxW = maxW
        self.maxV = maxV

        self.topic = topic
        self.data = CMDVel()
        self.pub = self.create_publisher(Twist, topic, 10)
        
        self.cli = self.create_client(TelloAction, 'tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()
        self.subscription = self.create_subscription(
            TelloResponse,
            '/tello_response',
            self.listener_callback,
            10)
        
        self.lock = threading.Lock()

        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)
        self.tello_response = 0

        self.thread.daemon = True
        self.state = 0
        self.start()
        
    def listener_callback(self, msg):
        self.tello_response = int(msg.rc)
 
    def publish (self):
        self.lock.acquire()
        tw = cmdvel2Twist(self.data)
        self.lock.release()
        
        #self.pub.publish(tw)
        
    def stop(self):
   
        self.kill_event.set()
        self.pub.unregister()

    def start (self):

        self.kill_event.clear()
        self.thread.start()
        
    def getMaxW(self):
        return self.maxW

    def getMaxV(self):
        return self.maxV
        

    def sendVelocities(self, vx, vy, vz, az):

        self.lock.acquire()
        self.data.vx = vx
        self.data.vy = vy
        self.data.vz = vz
        self.data.az = az
        self.lock.release()

    def sendV(self, v):

        self.sendVX(v)

    def sendL(self, l):

        self.sendVY(l)

    def sendW(self, w):

        self.sendAZ(w)

    def sendVX(self, vx):

        self.lock.acquire()
        self.data.vx = vx
        self.lock.release()
        tw = cmdvel2Twist(self.data)
        self.pub.publish(tw)

    def sendVY(self, vy):

        self.lock.acquire()
        self.data.vy = vy
        self.lock.release()
        tw = cmdvel2Twist(self.data)
        self.pub.publish(tw)

    def sendVZ(self, vz):

        self.lock.acquire()
        self.data.vz = vz
        self.lock.release()
        tw = cmdvel2Twist(self.data)
        self.pub.publish(tw)
        
    def sendAZ(self, az):
        self.lock.acquire()
        self.data.az = az
        self.lock.release()
        tw = cmdvel2Twist(self.data)
        self.pub.publish(tw)
        
    def send_request(self, cmd):
        self.req.cmd = cmd
        self.future = self.cli.call_async(self.req)
        
    def sendTwist(self, vx,vy,vz,az):
        self.data.vx = vx
        self.data.vy = vy 
        self.data.vz = vz
        self.data.az = az
        tw = cmdvel2Twist(self.data)
        self.pub.publish(tw)
        
        
    def takeoff(self):
        self.send_request('takeoff')
        #print("fin takeoff")
        self.state = 1
        
    def pause(self):
        self.send_request('stop')
        if (self.state == -1):
            self.state = 2
        
    def land(self):
        self.send_request('land')
        if (self.state == -1):
            self.state = 3
            
    def left(self, d):
        msg = "left " + str(d)
        self.send_request(msg)
        
    def right(self, d):
        msg = "right " + str(d)
        self.send_request(msg)
        
    def up(self, d):
        msg = "up " + str(d)
        self.send_request(msg)
        
    def forward(self, d):
        print("SE SUPONE QUE PALANTE")
        msg = "forward " + str(d)
        self.send_request(msg)
        
    def back(self, d):
        msg = "back " + str(d)
        self.send_request(msg)
        
    def turn_left(self, d):
        msg = "ccw " + str(d)
        print("SE SUPONE QUE GIRO")
        self.send_request(msg)
        #self.msg = "ccw " + str(d)
        #self.send_request(msg)
        
    def turn_right(self, d):
        msg = "cw " + str(d)
        self.send_request(msg)
        #self.msg = "cw " + str(d)
        #self.send_request(msg)
        
    def getState(self):
        return self.state
