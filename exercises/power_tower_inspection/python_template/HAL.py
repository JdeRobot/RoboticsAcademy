import numpy as np
import rclpy
import threading
import time
import sys

from hal_interfaces.general.camera import CameraNode
from jderobot_drones.drone_wrapper import DroneWrapper

IMG_WIDTH = 320
IMG_HEIGHT = 240
freq = 30.0


# Mutes exceptions
def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook

### HAL INIT ###

print("HAL initializing", flush=True)
if not rclpy.ok():
    rclpy.init()


CAM_FRONTAL_TOPIC = "/" + "drone0" + "/frontal_cam/image_raw"
CAM_VENTRAL_TOPIC = "/" + "drone0" + "/ventral_cam/image_raw"

drone = DroneWrapper()
frontal_camera_node = CameraNode(CAM_FRONTAL_TOPIC)
ventral_camera_node = CameraNode(CAM_VENTRAL_TOPIC)

# Spin nodes so that subscription callbacks load topic data
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(frontal_camera_node)
executor.add_node(ventral_camera_node)


def __auto_spin() -> None:
    while rclpy.ok():
        try:
            executor.spin_once(timeout_sec=0)
        except Exception:
            pass
        time.sleep(1 / freq)


executor_thread = threading.Thread(target=__auto_spin, daemon=True)
executor_thread.start()

### PAUSE / RESET HANDLING ###

# State management for pause/resume functionality
_hal_state = {
    "paused": False,
    "hold_position": None,
    "last_command_time": time.time(),
    "command_timeout": 0.5,  # seconds before auto-hold engages
    "initial_position": None,
    "lock": threading.Lock()
}


def __position_hold_thread():
    """Background thread that maintains position when paused or no commands received."""
    while rclpy.ok():
        time.sleep(0.1)
        with _hal_state["lock"]:
            if _hal_state["paused"] and _hal_state["hold_position"] is not None:
                # Hold position when paused
                pos = _hal_state["hold_position"]
                drone.set_cmd_pos(pos[0], pos[1], pos[2], pos[3])


# Start position hold thread
position_hold_thread = threading.Thread(target=__position_hold_thread, daemon=True)
position_hold_thread.start()


def pause():
    """Pause the drone and hold current position."""
    with _hal_state["lock"]:
        _hal_state["paused"] = True
        pos = drone.get_position()
        orientation = drone.get_orientation()
        if pos is not None and orientation is not None:
            # Store current position for holding
            yaw = orientation[2] if len(orientation) > 2 else 0.0
            _hal_state["hold_position"] = [pos[0], pos[1], pos[2], yaw]


def resume():
    """Resume from pause."""
    with _hal_state["lock"]:
        _hal_state["paused"] = False
        _hal_state["hold_position"] = None


def reset():
    """Reset drone to initial position."""
    with _hal_state["lock"]:
        _hal_state["paused"] = False
        _hal_state["hold_position"] = None
        # Return to initial position at safe altitude
        if _hal_state["initial_position"] is not None:
            init_pos = _hal_state["initial_position"]
            drone.set_cmd_pos(init_pos[0], init_pos[1], init_pos[2], init_pos[3])
        else:
            # Default reset position
            drone.set_cmd_pos(0.0, 0.0, 3.0, 0.0)


def _update_command_time():
    """Update the last command time and handle auto-pause detection."""
    with _hal_state["lock"]:
        _hal_state["last_command_time"] = time.time()
        if _hal_state["paused"]:
            # Auto-resume if new command received
            _hal_state["paused"] = False
            _hal_state["hold_position"] = None

### GETTERS ###


def get_frontal_image():
    image = frontal_camera_node.getImage()
    while image is None:
        image = frontal_camera_node.getImage()
    return image.data


def get_ventral_image():
    image = ventral_camera_node.getImage()
    while image is None:
        image = ventral_camera_node.getImage()
    return image.data


def get_position():
    pos = drone.get_position()
    return pos


def get_velocity():
    vel = drone.get_velocity()
    return vel


def get_yaw_rate():
    yaw_rate = drone.get_yaw_rate()
    return yaw_rate


def get_orientation():
    orientation = drone.get_orientation()
    return orientation


def get_roll():
    roll = drone.get_roll()
    return roll


def get_pitch():
    pitch = drone.get_pitch()
    return pitch


def get_yaw():
    yaw = drone.get_yaw()
    return yaw


def get_landed_state():
    state = drone.get_landed_state()
    return state


### SETTERS ###


def set_cmd_pos(x, y, z, az):
    _update_command_time()
    with _hal_state["lock"]:
        if _hal_state["paused"]:
            return
    drone.set_cmd_pos(x, y, z, az)


def set_cmd_vel(vx, vy, vz, az):
    _update_command_time()
    with _hal_state["lock"]:
        if _hal_state["paused"]:
            return
    drone.set_cmd_vel(vx, vy, vz, az)


def set_cmd_mix(vx, vy, z, az):
    """Set mixed velocity/position command.
    
    This function sends velocity commands for x and y, while maintaining
    a position setpoint for altitude (z) and yaw (az).
    
    Args:
        vx: Velocity in x direction (m/s)
        vy: Velocity in y direction (m/s)
        z: Target altitude/position in z (m)
        az: Target yaw angle (rad)
    
    Note: Due to controller behavior, small altitude drift may occur.
    For precise altitude hold, use set_cmd_pos() instead.
    """
    _update_command_time()
    with _hal_state["lock"]:
        if _hal_state["paused"]:
            return
    drone.set_cmd_mix(vx, vy, z, az)


def takeoff(h=3):
    """Takeoff to specified height.
    
    After takeoff, stores the initial position for reset functionality.
    """
    drone.takeoff(h)
    # Wait for takeoff to complete and store initial position
    time.sleep(0.5)
    pos = drone.get_position()
    orientation = drone.get_orientation()
    if pos is not None and orientation is not None:
        with _hal_state["lock"]:
            yaw = orientation[2] if len(orientation) > 2 else 0.0
            _hal_state["initial_position"] = [pos[0], pos[1], pos[2], yaw]
            print(f"HAL: Initial position set to {pos}", flush=True)


def land():
    _update_command_time()
    with _hal_state["lock"]:
        if _hal_state["paused"]:
            return
    drone.land()
