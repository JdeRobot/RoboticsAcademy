# the mouse. this one is pre-programmed (the student only writes the cat).
# nothing clever - it just flies a square over and over so the cat has
# something to chase. keep ESCAPE_SPEED below the cat's so it gets caught.
import HAL
import time
import math

TAKEOFF_HEIGHT = 3.0

ESCAPE_SPEED = 1.5
# square around the mouse's spawn (x=20, y=5), 10m a side
WAYPOINTS = [
    (25.0,  5.0, TAKEOFF_HEIGHT),
    (25.0, 10.0, TAKEOFF_HEIGHT),
    (20.0, 10.0, TAKEOFF_HEIGHT),
    (15.0, 10.0, TAKEOFF_HEIGHT),
    (15.0,  5.0, TAKEOFF_HEIGHT),
    (15.0,  0.0, TAKEOFF_HEIGHT),
    (20.0,  0.0, TAKEOFF_HEIGHT),
    (25.0,  0.0, TAKEOFF_HEIGHT),
]
REACH_THRESHOLD = 1.2   # how close counts as "reached this corner"


def distance(pos, target):
    return math.sqrt(
        (pos[0] - target[0]) ** 2 +
        (pos[1] - target[1]) ** 2 +
        (pos[2] - target[2]) ** 2
    )


HAL.takeoff(TAKEOFF_HEIGHT)

wp_index = 0

while True:
    pos = HAL.get_position()
    target = WAYPOINTS[wp_index]

    # close enough to this corner? move on to the next one (wrap around)
    if distance(pos, target) < REACH_THRESHOLD:
        wp_index = (wp_index + 1) % len(WAYPOINTS)
        target = WAYPOINTS[wp_index]

    # head straight at the current corner at a fixed speed
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    dz = target[2] - pos[2]
    norm = math.sqrt(dx**2 + dy**2 + dz**2) + 1e-6

    HAL.set_cmd_vel(
        ESCAPE_SPEED * dx / norm,
        ESCAPE_SPEED * dy / norm,
        ESCAPE_SPEED * dz / norm,
        0.0,
    )

    time.sleep(0.05)
