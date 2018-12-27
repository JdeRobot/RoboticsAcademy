import rospy
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Header
import threading
from math import pi as PI
from .threadPublisher import ThreadPublisher

class CMDVel ():

    def __init__(self):

        self.px = 0 # position in x
        self.py = 0 # position in y
        self.pz = 0 # position in z (altitude)
        self.vx = 0 # vel in x[m/s] (use this for V in wheeled robots)
        self.vy = 0 # vel in y[m/s]
        self.vz = 0 # vel in z[m/s]
        self.ax = 0 # angular vel in X axis [rad/s]
        self.ay = 0 # angular vel in X axis [rad/s]
        self.az = 0 # angular vel in Z axis [rad/s] (use this for W in wheeled robots)
        self.yaw = 0.0
        self.yaw_rate = 0.0 # yaw of the drone (use this for turn)
        self.timeStamp = 0 # Time stamp [s]


    def __str__(self):
        s = "CMDVel: {\n   vx: " + str(self.vx) + "\n   vy: " + str(self.vy)
        s = s + "\n   vz: " + str(self.vz) + "\n   ax: " + str(self.ax)
        s = s + "\n   ay: " + str(self.ay) + "\n   az: " + str(self.az)
        s = s + "\n   timeStamp: " + str(self.timeStamp)  + "\n}"
        return s

def cmdvel2PosTarget(vel):
    '''
    Translates from JderobotTypes CMDVel to ROS Twist.

    @param vel: JderobotTypes CMDVel to translate

    @type img: JdeRobotTypes.CMDVel

    @return a Twist translated from vel

    '''
    msg=PositionTarget(
        header=Header(
            stamp=rospy.Time.now(),
            frame_id=''),
    )
    msg.coordinate_frame = 8
    msg.type_mask = 1475
    msg.position.x = vel.px
    msg.position.y = vel.py
    msg.position.z = vel.pz
    msg.velocity.x = vel.vx
    msg.velocity.y = vel.vy
    msg.velocity.z = vel.vz
    msg.acceleration_or_force.x = vel.ax
    msg.acceleration_or_force.y = vel.ay
    msg.acceleration_or_force.z = vel.az
    msg.yaw = vel.yaw
    msg.yaw_rate = vel.yaw_rate
    return msg

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
        self.pub = rospy.Publisher(topic, PositionTarget, queue_size=1)
        self.lock = threading.Lock()

        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)

        self.thread.daemon = True
        self.start()

        self.using_event = threading.Event()
        self.using_event.set()

    def publish (self):
        '''
        Function to publish cmdvel.
        '''
        #print(self)
        #self.using_event.wait()

        self.lock.acquire()
        msg = cmdvel2PosTarget(self.vel)
        self.lock.release()
        self.pub.publish(msg)

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

    def pause (self):
        self.using_event.clear()


    def resume(self):
        self.using_event.set()


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

    def sendCMDVel (self,px,py,pz,vx,vy,vz,ax,ay,az,yaw,yaw_rate):
        self.lock.acquire()
        self.vel.px=px
        self.vel.py=py
        self.vel.pz=pz
        self.vel.vx=vx
        self.vel.vy=vy
        self.vel.vz=vz
        self.vel.ax=ax
        self.vel.ay=ay
        self.vel.az=az
        self.vel.yaw=yaw
        self.vel.yaw_rate=yaw_rate
        self.lock.release()
