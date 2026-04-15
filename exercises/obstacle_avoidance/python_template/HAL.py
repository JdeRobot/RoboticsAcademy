import rclpy
import threading
import sys

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode

# Hardware Abstraction Layer
IMG_WIDTH = 320
IMG_HEIGHT = 240
# {"name": "target10", "x": -47, "y": 45},


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

# ROS2 init
if not rclpy.ok():
    rclpy.init(args=None)
    rclpy.create_node("HAL")

pose3d = OdometryNode("/odom")
motors = MotorsNode("/cmd_vel", 4, 0.3)
laser = LaserNode("/f1/laser/scan")

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor.add_node(laser)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

print("HAL-Nodes Thread Started")


def getPose3d():
    return pose3d.getPose3d()


def getLaserData():
    laser_data = laser.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser.getLaserData()
    return laser_data


def setV(velocity):
    motors.sendV(float(velocity))


def setW(velocity):
    motors.sendW(float(velocity))

# Variables globales para recordar la última velocidad enviada
_last_v = 0.0
_last_w = 0.0

def setVW(v, w, dt=0.1, v_max=2.0, w_max=2.0, a_v_max=2, a_w_max=2):
    global _last_v, _last_w

    # 1. Limitar velocidades máximas
    v = max(min(v, v_max), -v_max)
    w = max(min(w, w_max), -w_max)

    # 2. Limitar aceleración (cambio respecto al último valor)
    dv = v - _last_v
    dw = w - _last_w

    dv_max = a_v_max * dt
    dw_max = a_w_max * dt

    dv = max(min(dv, dv_max), -dv_max)
    dw = max(min(dw, dw_max), -dw_max)

    v_limited = _last_v + dv
    w_limited = _last_w + dw

    # 3. Enviar a motores
    motors.sendV(float(v_limited))
    motors.sendW(float(w_limited))

    # 4. Guardar valores actuales
    _last_v = v_limited
    _last_w = w_limited
