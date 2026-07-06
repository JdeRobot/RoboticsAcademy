import HAL
import time
import math

TAKEOFF_HEIGHT = 3.0
CHASE_SPEED = 2.0  # keep this above the mouse's 1.5 or the cat never catches up
SLOW_RADIUS = 3.0  # once we're this close, ease off so we don't overshoot

# get off the ground first
HAL.takeoff(TAKEOFF_HEIGHT)

# then just keep aiming at wherever the mouse is right now
while True:
    cat = HAL.get_position()
    mouse = HAL.get_mouse_position()

    dx = mouse[0] - cat[0]
    dy = mouse[1] - cat[1]
    dz = TAKEOFF_HEIGHT - cat[2]  # hold our height

    dist = math.sqrt(dx * dx + dy * dy)

    # go full speed when the mouse is far, slow down as we close in - otherwise
    # the cat blows past it and ends up wobbling back and forth around it
    speed = CHASE_SPEED * min(1.0, dist / SLOW_RADIUS)

    if dist > 1e-3:
        vx = speed * dx / dist
        vy = speed * dy / dist
    else:
        vx = vy = 0.0

    HAL.set_cmd_vel(vx, vy, dz, 0.0)
    time.sleep(0.05)
