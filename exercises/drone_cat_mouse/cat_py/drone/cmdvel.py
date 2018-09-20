import rospy
from geometry_msgs.msg import TwistStamped
import threading
from math import pi as PI
from .threadPublisher import ThreadPublisher

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

def cmdvel2Twist(vel):
    '''
    Translates from JderobotTypes CMDVel to ROS Twist. 

    @param vel: JderobotTypes CMDVel to translate

    @type img: JdeRobotTypes.CMDVel

    @return a Twist translated from vel

    '''
    tw = TwistStamped()
    tw.twist.linear.x = vel.vx
    tw.twist.linear.y = vel.vy
    tw.twist.linear.z = vel.vz
    tw.twist.angular.x = vel.ax
    tw.twist.angular.y = vel.ay
    tw.twist.angular.z = vel.az

    return tw

class PublisherCMDVel:
    '''
        ROS CMDVel Publisher. CMDVel Client to Send CMDVel to ROS nodes.
    '''
    def __init__(self, topic):
        '''
        PublisherCMDVel Constructor.

        @param topic: ROS topic to publish
        
        @type topic: String

        '''
        rospy.init_node("ss")
        self.topic = topic
        self.vel = CMDVel()
        self.pub = self.pub = rospy.Publisher(topic, TwistStamped, queue_size=1)
        self.lock = threading.Lock()

        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)

        self.thread.daemon = True
        self.start()
 
    def publish (self):
        '''
        Function to publish cmdvel. 
        '''
        self.lock.acquire()
        tw = cmdvel2Twist(self.vel)
        self.lock.release()
        self.pub.publish(tw)
        
    def stop(self):
        '''
        Stops (Unregisters) the client. If client is stopped you can not start again, Threading.Thread raised error

        '''
        self.kill_event.set()
        self.pub.unregister()

    def start (self):
        '''
        Starts (Subscribes) the client. If client is stopped you can not start again, Threading.Thread raised error

        '''
        self.kill_event.clear()
        self.thread.start()
        
        

    def sendVelocities(self):
        '''
        Sends CMDVel.

        @param vel: CMDVel to publish
        
        @type vel: CMDVel

        '''
        self.lock.acquire()
        #self.vel = vel
        self.lock.release()


    def setVX(self, vx):
        '''
        Sends VX velocity.

        @param vx: VX velocity
        
        @type vx: float

        '''
        self.lock.acquire()
        self.vel.vx = vx
        self.lock.release()

    def setVY(self, vy):
        '''
        Sends VY velocity.

        @param vy: VY velocity
        
        @type vy: float

        '''
        self.lock.acquire()
        self.vel.vy = vy
        self.lock.release()


    def setVZ(self,vz):
        '''
        Sends VZ velocity.

        @param vz: VZ velocity
        
        @type vz: float

        '''
        self.lock.acquire()
        self.vel.vz=vz
        self.lock.release()
    
    def setAngularZ(self, az):
        '''
        Sends AZ velocity.

        @param az: AZ velocity
        
        @type az: float

        '''
        self.lock.acquire()
        self.vel.az = az
        self.lock.release()

    def setAngularX(self,ax):
        '''
        Sends AX velocity.

        @param ax: AX velocity
        
        @type ax: float

        '''
        self.lock.acquire()
        self.vel.ax=ax
        self.lock.release()
        
    def setAngularY(self,ay):
        '''
        Sends AY velocity.

        @param ay: AY velocity
        
        @type ay: float

        '''
        self.lock.acquire()
        self.vel.ay=ay
        self.lock.release()

    def setYaw(self,yaw):
       self.setAngularZ(yaw)

    def setRoll(self,roll):
        self.setAngularX(roll)
        
    def setPitch(self,pitch):
        self.setAngularY(pitch)

    def sendCMD (self, vel):
        '''
        Sends CMDVel.

        @param vel: CMDVel to publish
        
        @type vel: CMDVel

        '''
        self.lock.acquire()
        self.vel = vel
        self.lock.release()

    def sendCMDVel (self, vx,vy,vz,ax,ay,az):
        self.lock.acquire()
        self.vel.vx=vx
        self.vel.vy=vy
        self.vel.vz=vz
        self.vel.ax=ax
        self.vel.ay=ay
        self.vel.az=az
        self.lock.release()


