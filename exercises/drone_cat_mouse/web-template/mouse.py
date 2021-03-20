import threading

from drone_wrapper import DroneWrapper
from interfaces.autonomous_mouse import path_obfuscated


class Mouse:
    def __init__(self):
        self.mouse = DroneWrapper(name="rqt", ns="mouse/")

    # Explicit initialization functions
    # Class method, so user can call it without instantiation
    @classmethod
    def initRobot(cls):
        new_instance = cls()
        return new_instance

    def takeoff(self):
        self.mouse.takeoff(h=5)

    def start_mouse(self, path_level):
        self.thread = threading.Thread(target=self.autonomous_mouse, args=[path_level])
        self.thread.start()

    def land(self):
        self.mouse.land()

    def autonomous_mouse(self, path_level):
        while True:
            v = path_obfuscated(path_level)
            if v is not None:
                vx, vy, vz, yaw = v
                self.mouse.set_cmd_vel(vx, vy, vz, yaw)
            else:
                print("[Mouse] Path {} not available".format(path_level))
                break
