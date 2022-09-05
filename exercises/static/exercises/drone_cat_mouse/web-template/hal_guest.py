import rospy
import cv2
import threading
import time
from datetime import datetime

from drone_wrapper import DroneWrapper
from shared.image import SharedImage
from shared.value import SharedValue

# Hardware Abstraction Layer
class HAL:
    IMG_WIDTH = 320
    IMG_HEIGHT = 240

    def __init__(self):
        rospy.init_node("HAL_mouse")

        # Shared memory variables
        self.shared_frontal_image = SharedImage("halfrontalimageguest")
        self.shared_ventral_image = SharedImage("halventralimageguest")
        self.shared_x = SharedValue("xguest")
        self.shared_y = SharedValue("yguest")
        self.shared_z = SharedValue("zguest")
        self.shared_takeoff_z = SharedValue("sharedtakeoffzguest")
        self.shared_az = SharedValue("azguest")
        self.shared_azt = SharedValue("aztguest")
        self.shared_vx = SharedValue("vxguest")
        self.shared_vy = SharedValue("vyguest")
        self.shared_vz = SharedValue("vzguest")
        self.shared_landed_state = SharedValue("landedstateguest")
        self.shared_position = SharedValue("positionguest")
        self.shared_velocity = SharedValue("velocityguest")
        self.shared_orientation = SharedValue("orientationguest")
        self.shared_roll = SharedValue("rollguest")
        self.shared_pitch = SharedValue("pitchguest")
        self.shared_yaw = SharedValue("yawguest")
        self.shared_yaw_rate = SharedValue("yawrateguest")

        self.image = None
        self.mouse = DroneWrapper(name="rqt", ns="/iris1/")
        
        # Update thread
        self.thread = ThreadHAL(self.update_hal)

    # Function to start the update thread
    def start_thread(self):
        self.thread.start()

    # Get Image from ROS Driver Camera
    def get_frontal_image(self):
        image = self.mouse.get_frontal_image()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.shared_frontal_image.add(image_rgb)

    def get_ventral_image(self):
        image = self.mouse.get_ventral_image()
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.shared_ventral_image.add(image_rgb)

    def get_position(self):
        pos = self.mouse.get_position()
        self.shared_position.add(pos,type_name="list")

    def get_velocity(self):
        vel = self.mouse.get_velocity()
        self.shared_velocity.add(vel ,type_name="list")

    def get_yaw_rate(self):
        yaw_rate = self.mouse.get_yaw_rate()
        self.shared_yaw_rate.add(yaw_rate)

    def get_orientation(self):
        orientation = self.mouse.get_orientation()
        self.shared_orientation.add(orientation ,type_name="list")

    def get_roll(self):
        roll = self.mouse.get_roll()
        self.shared_roll.add(roll)

    def get_pitch(self):
        pitch = self.mouse.get_pitch()
        self.shared_pitch.add(pitch)

    def get_yaw(self):
        yaw = self.mouse.get_yaw()
        self.shared_yaw.add(yaw)

    def get_landed_state(self):
        state = self.mouse.get_landed_state()
        self.shared_landed_state.add(state)

    def set_cmd_pos(self):
        x = self.shared_x.get()
        y = self.shared_y.get()
        z = self.shared_z.get()
        az = self.shared_az.get()

        self.mouse.set_cmd_pos(x, y, z, az)

    def set_cmd_vel(self):
        vx = self.shared_vx.get()
        vy = self.shared_vy.get()
        vz = self.shared_vz.get()
        az = self.shared_azt.get()
        self.mouse.set_cmd_vel(vx, vy, vz, az)

    def set_cmd_mix(self):
        vx = self.shared_vx.get()
        vy = self.shared_vy.get()
        z = self.shared_z.get()
        az = self.shared_azt.get()
        self.mouse.set_cmd_mix(vx, vy, z, az)

    def takeoff(self):
        h = self.shared_takeoff_z.get()
        self.mouse.takeoff(h)

    def land(self):
        self.mouse.land()

    def update_hal(self):
        self.get_frontal_image()
        self.get_ventral_image()
        self.get_position()
        self.get_velocity()
        self.get_yaw_rate()
        self.get_orientation()
        self.get_pitch()
        self.get_roll()
        self.get_yaw()
        self.get_landed_state()
        self.set_cmd_pos()
        self.set_cmd_vel()
        self.set_cmd_mix()

    # Destructor function to close all fds
    def __del__(self):
        self.shared_frontal_image.close()
        self.shared_ventral_image.close()
        self.shared_x.close()
        self.shared_y.close()
        self.shared_z.close()
        self.shared_takeoff_z.close()
        self.shared_az.close()
        self.shared_azt.close()
        self.shared_vx.close()
        self.shared_vy.close()
        self.shared_vz.close()
        self.shared_landed_state.close()
        self.shared_position.close()
        self.shared_velocity.close()
        self.shared_orientation.close()
        self.shared_roll.close()
        self.shared_pitch.close()
        self.shared_yaw.close()
        self.shared_yaw_rate.close()

class ThreadHAL(threading.Thread):
    def __init__(self, update_function):
        super(ThreadHAL, self).__init__()
        self.time_cycle = 80
        self.update_function = update_function

    def run(self):
        while(True):
            start_time = datetime.now()

            self.update_function()

            finish_time = datetime.now()

            dt = finish_time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0

            if(ms < self.time_cycle):
                time.sleep((self.time_cycle - ms) / 1000.0)