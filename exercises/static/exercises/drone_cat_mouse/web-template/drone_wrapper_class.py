#!/usr/bin/env python

import rospy
import tf
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, PositionTarget, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, \
    ParamSet, ParamGet

EPSILON = 0.01
CMD = None


class DroneWrapper:
    def state_cb(self, msg):
        self.state = msg
        rospy.logdebug('State updated')

    def extended_state_cb(self, msg):
        self.extended_state = msg
        rospy.logdebug('Extended State updated')

        self.rqt_extended_state_publisher.publish(self.extended_state)

    def pose_stamped_cb(self, msg):
        self.pose_stamped = msg
        rospy.logdebug('Pose updated')

        self.rqt_pose_publisher.publish(self.pose_stamped)

    def vel_body_stamped_cb(self, msg):
        self.vel_body_stamped = msg
        rospy.logdebug('Velocity (body) updated')

        self.rqt_velocity_body_publisher.publish(self.vel_body_stamped)

    def global_position_cb(self, msg):
        self.global_position = msg
        rospy.logdebug('Global position updated')

    def cam_frontal_cb(self, msg):
        self.frontal_image = msg
        rospy.logdebug('Frontal image updated')

        self.rqt_cam_frontal_publisher.publish(self.frontal_image)

    def cam_ventral_cb(self, msg):
        self.ventral_image = msg
        rospy.logdebug('Ventral image updated')

        self.rqt_cam_ventral_publisher.publish(self.ventral_image)

    def stay_armed_stay_offboard_cb(self, event):
        if self.state.mode != 'OFFBOARD':
            if self.request_mode('OFFBOARD'):
                rospy.loginfo("OFFBOARD requested")
        elif not self.state.armed:
            if self.arm(True):
                rospy.loginfo("Vehicle Armed")

    def get_frontal_image(self):
        return self.bridge.imgmsg_to_cv2(self.frontal_image)

    def get_ventral_image(self):
        return self.bridge.imgmsg_to_cv2(self.ventral_image)

    def get_position(self):
        return np.array([self.pose_stamped.pose.position.x,
                         self.pose_stamped.pose.position.y,
                         self.pose_stamped.pose.position.z])

    def get_velocity(self):
        return np.array([self.vel_body_stamped.twist.linear.x,
                         self.vel_body_stamped.twist.linear.y,
                         self.vel_body_stamped.twist.linear.z])

    def get_yaw_rate(self):
        return self.vel_body_stamped.twist.angular.z

    def get_orientation(self):
        return np.array(tf.transformations.euler_from_quaternion([self.pose_stamped.pose.orientation.x,
                                                                  self.pose_stamped.pose.orientation.y,
                                                                  self.pose_stamped.pose.orientation.z,
                                                                  self.pose_stamped.pose.orientation.w]))

    def get_roll(self):
        return self.get_orientation()[0]

    def get_pitch(self):
        return self.get_orientation()[1]

    def get_yaw(self):
        return self.get_orientation()[2]

    def get_landed_state(self):
        return self.extended_state.landed_state

    def param_set(self, param, value):
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)

        rospy.wait_for_service(self.ns + '/mavros/param/set')
        try:
            set_param = rospy.ServiceProxy(self.ns + '/mavros/param/set', ParamSet)
            resp = set_param(param_id=param, value=val)
            print("setmode send ok", resp.success)
        except rospy.ServiceException as e:
            print("Failed SetMode:", e)

    def param_get(self, param):
        try:
            get_param = rospy.ServiceProxy(self.ns + 'mavros/param/get', ParamGet)
            resp = get_param(param_id=param)
            print("setmode send ok", resp.success)
        except rospy.ServiceException as e:
            print("Failed SetMode:", e)
            return None

        if resp.value.integer != 0:
            return resp.value.integer
        elif resp.value.real != 0.0:
            return resp.value.real
        else:
            return 0

    def arm(self, value=True):
        req = CommandBoolRequest()
        req.value = value
        if self.arm_client(req).success:
            rospy.loginfo('Arming/Disarming successful')
            return True
        else:
            rospy.logwarn('Arming/Disarming unsuccessful')
            return False

    def request_mode(self, mode='OFFBOARD'):
        rospy.sleep(2)
        rospy.loginfo('Current mode: %s', self.state.mode)
        req = SetModeRequest()
        req.custom_mode = mode
        if self.mode_client(req).mode_sent:
            rospy.loginfo('Mode change request successful')
            return True
        else:
            rospy.logwarn('Mode change request unsuccessful')
            return False

    def set_cmd_pos(self, x=0, y=0, z=0, az=0):
        self.setpoint_raw.coordinate_frame = 8
        self.setpoint_raw.yaw = az

        self.posx = x
        self.posy = y
        self.height = z
        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.setpoint_raw.position.x = x
        self.setpoint_raw.position.y = y
        self.setpoint_raw.position.z = z

        global CMD
        CMD = 0  # POS
        self.setpoint_raw.type_mask = 3064  # xyz yaw

        self.setpoint_raw_publisher.publish(self.setpoint_raw)

    def set_cmd_vel(self, vx=0, vy=0, vz=0, az=0):
        self.setpoint_raw.coordinate_frame = 8
        self.setpoint_raw.yaw_rate = az

        self.posx = self.pose_stamped.pose.position.x
        self.posy = self.pose_stamped.pose.position.y
        self.height = self.pose_stamped.pose.position.z
        self.vx = -vy
        self.vy = vx
        self.vz = vz

        global CMD
        CMD = 1  # VEL

        if abs(vx) <= EPSILON and abs(vy) <= EPSILON:
            self.is_xy = True
        else:
            self.setpoint_raw.velocity.x = -vy
            self.setpoint_raw.velocity.y = vx

            self.is_xy = False

        if abs(vz) <= EPSILON:
            self.is_z = True
        else:
            self.setpoint_raw.velocity.z = vz
            self.is_z = False

        if self.is_xy:
            if self.is_z:
                self.setpoint_raw.type_mask = 2040  # xyz yaw_rate
                # self.setpoint_raw.type_mask = 3064 # xyz yaw
            else:
                self.setpoint_raw.type_mask = 1991  # vx vy vy yaw_rate
                # self.setpoint_raw.type_mask = 3015  # vx vy vz yaw
                # self.setpoint_raw.type_mask = 3036 # x y vz yaw -> NOT SUPPORTED
        else:
            if self.is_z:
                self.setpoint_raw.type_mask = 1987  # vx vy vz z yaw_rate
                # self.setpoint_raw.type_mask = 2019  # vx vy z yaw_rate -> NOT SUPPORTED
                # self.setpoint_raw.type_mask = 3011  # vx vy vz z yaw
                # self.setpoint_raw.type_mask = 3043  # vx vy z yaw -> NOT SUPPORTED
            else:
                self.setpoint_raw.type_mask = 1991  # vx vy vy yaw_rate
                # self.setpoint_raw.type_mask = 3015  # vx vy vz yaw

        self.setpoint_raw_publisher.publish(self.setpoint_raw)

    def set_cmd_mix(self, vx=0, vy=0, z=0, az=0):
        self.setpoint_raw.coordinate_frame = 8
        self.setpoint_raw.yaw_rate = az

        self.posx = self.pose_stamped.pose.position.x
        self.posy = self.pose_stamped.pose.position.y
        self.height = z
        self.vx = -vy
        self.vy = vx
        self.vz = 0

        self.setpoint_raw.position.z = z
        self.setpoint_raw.velocity.x = -vy
        self.setpoint_raw.velocity.y = vx

        global CMD
        CMD = 2  # MIX
        self.setpoint_raw.type_mask = 1987  # vx vy vz z yaw_rate

        self.setpoint_raw_publisher.publish(self.setpoint_raw)

    def repeat_setpoint_raw(self, event):
        self.setpoint_raw.coordinate_frame = 8

        self.setpoint_raw.position.x = self.posx
        self.setpoint_raw.position.y = self.posy
        self.setpoint_raw.position.z = self.height
        self.setpoint_raw.velocity.x = self.vx
        self.setpoint_raw.velocity.y = self.vy
        self.setpoint_raw.velocity.z = self.vz

        if CMD == 0:  # POS
            self.setpoint_raw.type_mask = 3064  # xyz yaw
        elif CMD == 1:  # VEL
            if self.is_xy:
                if self.is_z:
                    self.setpoint_raw.type_mask = 2040  # xyz yaw_rate
                else:
                    self.setpoint_raw.type_mask = 1991  # vx vy vy yaw_rate
            else:
                if self.is_z:
                    self.setpoint_raw.type_mask = 1987  # vx vy vz z yaw_rate
                else:
                    self.setpoint_raw.type_mask = 1991  # vx vy vy yaw_rate
        elif CMD == 2:  # MIX
            self.setpoint_raw.type_mask = 1987  # vx vy vz z yaw_rate
        else:
            self.setpoint_raw.type_mask = 3064  # xyz yaw
            print("[CMD error]: Mask set to position control")

        self.setpoint_raw_publisher.publish(self.setpoint_raw)

    def hold_setpoint_raw(self):
        if not self.setpoint_raw_flag:
            self.setpoint_raw_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_setpoint_raw)

    def takeoff(self, h=3):
        self.set_cmd_pos(0, 0, 0, 0)
        self.hold_setpoint_raw()
        self.arm(True)
        self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)
        req = CommandTOLRequest()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = self.global_position.latitude
        req.longitude = self.global_position.longitude
        req.altitude = h
        self.takeoff_client(req)
        if self.takeoff_client(req).success:
            rospy.loginfo('Takeoff successful')
            return True
        else:
            rospy.logwarn('Takeoff unsuccessful')
            return False

    def take_control(self):
        self.set_cmd_pos(0, 0, 0, 0)
        self.hold_setpoint_raw()
        self.arm(True)
        self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(3), self.stay_armed_stay_offboard_cb)

    def land(self):
        self.setpoint_raw_timer.shutdown()
        self.stay_armed_stay_offboard_timer.shutdown()
        req = CommandTOLRequest()
        req.latitude = self.global_position.latitude
        req.longitude = self.global_position.longitude
        self.land_client(req)

    def __init__(self, name='drone', ns='', verbose=False):
        if name != 'rqt':
            if verbose:
                rospy.init_node(name, anonymous=True, log_level=rospy.DEBUG)
            else:
                rospy.init_node(name)
        self.ns = ns

        self.state = State()
        self.extended_state = ExtendedState()
        self.pose_stamped = PoseStamped()
        self.vel_body_stamped = TwistStamped()
        self.rate = rospy.Rate(20)
        self.setpoint_raw = PositionTarget()
        self.setpoint_raw_flag = False
        self.vz_factor = 0.4
        self.bridge = CvBridge()

        self.is_z = False
        self.is_xy = False

        self.posx = 0
        self.posy = 0
        self.height = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.setpoint_raw_timer = rospy.Timer(rospy.Duration(nsecs=50000000), self.repeat_setpoint_raw)
        self.setpoint_raw_timer.shutdown()
        self.stay_armed_stay_offboard_timer = rospy.Timer(rospy.Duration(5), self.stay_armed_stay_offboard_cb)
        self.stay_armed_stay_offboard_timer.shutdown()

        rospy.wait_for_service(self.ns + 'mavros/cmd/arming')
        self.arm_client = rospy.ServiceProxy(ns + 'mavros/cmd/arming', CommandBool)
        rospy.wait_for_service(self.ns + 'mavros/set_mode')
        self.mode_client = rospy.ServiceProxy(ns + 'mavros/set_mode', SetMode)
        rospy.wait_for_service(self.ns + 'mavros/cmd/takeoff')
        self.takeoff_client = rospy.ServiceProxy(ns + 'mavros/cmd/takeoff', CommandTOL)
        rospy.wait_for_service(self.ns + 'mavros/cmd/land')
        self.land_client = rospy.ServiceProxy(ns + 'mavros/cmd/land', CommandTOL)

        self.rqt_extended_state_publisher = rospy.Publisher(self.ns + 'drone_wrapper/extended_state', ExtendedState,
                                                            queue_size=1)
        self.rqt_pose_publisher = rospy.Publisher(self.ns + 'drone_wrapper/local_position/pose', PoseStamped,
                                                  queue_size=1)
        self.rqt_velocity_body_publisher = rospy.Publisher(self.ns + 'drone_wrapper/local_position/velocity_body',
                                                           TwistStamped, queue_size=1)
        self.rqt_cam_frontal_publisher = rospy.Publisher(self.ns + 'drone_wrapper/cam_frontal/image_raw', Image,
                                                         queue_size=1)
        self.rqt_cam_ventral_publisher = rospy.Publisher(self.ns + 'drone_wrapper/cam_ventral/image_raw', Image,
                                                         queue_size=1)

        rospy.Subscriber(self.ns + 'mavros/state', State, self.state_cb)
        rospy.Subscriber(self.ns + 'mavros/extended_state', ExtendedState, self.extended_state_cb)
        rospy.Subscriber(self.ns + 'mavros/local_position/pose', PoseStamped, self.pose_stamped_cb)
        rospy.Subscriber(self.ns + 'mavros/local_position/velocity_body', TwistStamped, self.vel_body_stamped_cb)
        rospy.Subscriber(self.ns + 'mavros/global_position/global', NavSatFix, self.global_position_cb)
        cam_frontal_topic = rospy.get_param('cam_frontal_topic', '/iris/cam_frontal/image_raw')
        cam_ventral_topic = rospy.get_param('cam_ventral_topic', '/iris/cam_ventral/image_raw')
        rospy.Subscriber(cam_frontal_topic, Image, self.cam_frontal_cb)
        rospy.Subscriber(cam_ventral_topic, Image, self.cam_ventral_cb)

        self.setpoint_raw_publisher = rospy.Publisher(self.ns + 'mavros/setpoint_raw/local', PositionTarget,
                                                      queue_size=1)
