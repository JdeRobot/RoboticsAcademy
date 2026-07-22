import HAL
import WebGUI
import time

# You are the cat. Chase the mouse drone.
#
# Sensing:  HAL.get_position()        -> your position       [x, y, z]
#           HAL.get_mouse_position()  -> the mouse position  [x, y, z]
#           HAL.get_frontal_image()   -> your frontal camera
#           HAL.get_ventral_image()   -> your ventral camera
# Moving:   HAL.takeoff(h), HAL.land()
#           HAL.set_cmd_vel(vx, vy, vz, yaw_rate)
# Showing:  WebGUI.showImage(img)      -> right panel
#           WebGUI.showLeftImage(img)  -> left panel

SPEED = 3.0

HAL.takeoff(3)

while True:
    cat = HAL.get_position()
    mouse = HAL.get_mouse_position()

    # Show both of the cat's cameras: frontal on the right, ventral on the left.
    WebGUI.showImage(HAL.get_frontal_image())
    WebGUI.showLeftImage(HAL.get_ventral_image())

    # Drive straight at the mouse. Improve it: lead the target, control speed.
    dx = mouse[0] - cat[0]
    dy = mouse[1] - cat[1]
    dz = mouse[2] - cat[2]
    dist = (dx * dx + dy * dy + dz * dz) ** 0.5 + 1e-6

    HAL.set_cmd_vel(SPEED * dx / dist, SPEED * dy / dist, SPEED * dz / dist, 0.0)
    time.sleep(0.05)
