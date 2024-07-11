import json
import math
import matplotlib.pyplot as plt

from gui_interfaces.general.threading_gui import ThreadingGUI
from map import Map
from HAL import getPose3d
from console import start_console

# Graphical User Interface Class

class GUI(ThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Payload vars
        self.payload = {'map': '', 'user': '', 'particles': ''}
        self.init_coords = (171, 63)
        self.start_coords = (201, 85.5)
        self.map = Map(getPose3d)

        # Particles
        self.particles = []
        # User position
        self.user_position = (0, 0)
        self.user_angle = (0, 0)

        self.start()

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        # Payload Map Message
        pos_message = self.map.getRobotCoordinates()
        if pos_message == self.init_coords:
            pos_message = self.start_coords
        ang_message = self.map.getRobotAngle()
        pos_message = str(pos_message + ang_message)
        self.payload["map"] = pos_message

        # Payload User Message
        pos_message_user = self.user_position
        ang_message_user = self.user_angle
        pos_message_user = pos_message_user + ang_message_user
        pos_message_user = str(pos_message_user)
        self.payload["user"] = pos_message_user

        # Payload Particles Message
        if self.particles:
            self.payload["particles"] = json.dumps(self.particles)
        else:
            self.payload["particles"] = json.dumps([])

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def showPosition(self, x, y, angle):
        angle += math.pi
        ay = math.cos(-angle) - math.sin(-angle)
        ax = math.sin(-angle) + math.cos(-angle)
        scale_y = 15
        offset_y = 63
        y = scale_y * y + offset_y
        scale_x = -30
        offset_x = 171
        x = scale_x * x + offset_x
        self.user_position = x, y
        self.user_angle = (ax, ay)

    def showParticles(self, particles):
        if particles:
            self.particles = particles
            scale_y = 15
            offset_y = 63
            scale_x = -30
            offset_x = 171
            for particle in self.particles:
                particle[1] = scale_y * particle[1] + offset_y
                particle[0] = scale_x * particle[0] + offset_x
        else:
            self.particles = []

    def getMap(self, url):
        return plt.imread(url)

    def poseToMap(self, x_prime, y_prime, yaw_prime):
        scale_x = 1024 / 9.885785
        offset_x = 5.650662
        x = 1181 - (x_prime + offset_x) * scale_x
        scale_y = 1024 / 9.75819
        offset_y = 4.088577
        y = 24 + (y_prime + offset_y) * scale_y
        yaw = yaw_prime
        return [x, y, yaw]

    def mapToPose(self, x, y, yaw):
        scale_x = 1024 / 9.885785
        offset_x = 5.650662
        x = ((1181 - x) / scale_x) - offset_x
        scale_y = 1024 / 9.75819
        offset_y = 4.088577
        y = ((-24 + y) / scale_y) - offset_y
        return [x, y, yaw]

host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()

# Expose to the user
def showPosition(x, y, angle):
    gui.showPosition(x, y, angle)

def showParticles(particles):
    gui.showParticles(particles)

def getMap(url):
    return gui.getMap(url)

def poseToMap(x_prime, y_prime, yaw_prime):
    return gui.poseToMap(x_prime, y_prime, yaw_prime)

def mapToPose(x, y, yaw):
    return gui.mapToPose(x, y, yaw)