# The mouse (target). Pre-programmed - the student only writes the cat.
#
# NOTE ON COMMUNICATION: this process and the cat process never talk directly.
# They only "see" each other through the simulator: HAL.get_cat_position() here
# reads the cat's ground-truth pose off a ROS topic, and the cat's
# HAL.get_mouse_position() reads ours.
#
# TODO: difficulty levels. Only one behaviour is wired up for now (a fast lap
# around the gates that jukes when the cat closes in). The multi-level version
# is almost ready - slower/predictable laps for pure pursuit, faster racing
# lines for proportional navigation, plus square / spiral / figure-8 / zig-zag
# paths - and will be pushed in a follow-up.
import HAL
import time
import math

# Speeds and gains for the single behaviour we ship today.
V_MAX = 4.6
V_MIN = 1.8
CORNER_K = 1.4  # How hard to brake into a corner
ACCEL_MAX = 7.0
APPROACH = 9.0  # Start braking this far from the next gate
GATE_RADIUS = 4.5  # How close counts as "reached this gate"
KYAW = 2.0

# Dodging: when the cat gets inside THREAT_RADIUS the mouse pushes sideways off
# its line, flipping the side it jukes to every JUKE_FLIP seconds.
THREAT_RADIUS = 8.0
DODGE_GAIN = 6.0
JUKE_FLIP = 0.9

TAKEOFF_HEIGHT = 10.0

# Waypoints (x, y, z) anchored in the open city box (centre 15, 0), at ~10 m.
GATES = [
    (30.0, -12.0, 10.0),
    (30.0, 12.0, 12.0),
    (15.0, 15.0, 10.5),
    (0.0, 12.0, 12.5),
    (0.0, -12.0, 10.0),
    (15.0, -15.0, 11.5),
]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def dist_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist3(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


HAL.takeoff(TAKEOFF_HEIGHT)

gi = 0
vx = vy = vz = 0.0
t0 = time.time()
juke_sign = 1.0
last_flip = 0.0

recover_height = TAKEOFF_HEIGHT
checkpoint = HAL.get_position()
last_progress = time.time()
recovering = False
cmd_speed = 0.0
crashed = False

while True:
    t = time.time() - t0
    pos = HAL.get_position()
    cat = HAL.get_cat_position()
    now = time.time()

    # Caught: fall to the ground and stay there.
    if crashed or dist3(pos, cat) < 1.8:
        crashed = True
        HAL.set_cmd_vel(0.0, 0.0, -2.5 if pos[2] > 0.4 else 0.0, 0.0)
        time.sleep(0.05)
        continue

    # Recovery: knocked down or pinned, so climb back up.
    moving_intent = cmd_speed > 0.5
    if dist3(pos, checkpoint) > 0.4 or not moving_intent:
        checkpoint = pos
        last_progress = now
    stuck = (now - last_progress > 2.5) or (pos[2] < 1.5)
    if stuck and not recovering:
        recovering = True
        recover_height = min(10.0, recover_height + 0.5)
    if recovering:
        HAL.set_cmd_vel(0.0, 0.0, 1.5, 0.0)
        if pos[2] >= recover_height - 0.2:
            recovering = False
            checkpoint = pos
            last_progress = now
        vx = vy = vz = 0.0
        cmd_speed = 0.0
        time.sleep(0.05)
        continue

    # Steer to the current gate, braking into the corner, then advance.
    gate = GATES[gi]
    nxt = GATES[(gi + 1) % len(GATES)]
    if dist_xy(pos, gate) < GATE_RADIUS:
        gi = (gi + 1) % len(GATES)
        gate = GATES[gi]
        nxt = GATES[(gi + 1) % len(GATES)]

    to = (gate[0] - pos[0], gate[1] - pos[1], gate[2] - pos[2])
    dg = math.sqrt(to[0] ** 2 + to[1] ** 2 + to[2] ** 2) + 1e-6
    dir_in = (to[0] / dg, to[1] / dg, to[2] / dg)

    leg = (nxt[0] - gate[0], nxt[1] - gate[1])
    lg = math.hypot(leg[0], leg[1]) + 1e-6
    dot = (dir_in[0] * leg[0] + dir_in[1] * leg[1]) / lg
    turn = math.acos(clamp(dot, -1.0, 1.0))
    corner = 1.0 / (1.0 + CORNER_K * turn)
    prox = clamp((APPROACH - dist_xy(pos, gate)) / APPROACH, 0.0, 1.0)
    speed = max(V_MIN, V_MAX * (1.0 - prox * (1.0 - corner)))

    tgt_vx = dir_in[0] * speed
    tgt_vy = dir_in[1] * speed
    tgt_vz = dir_in[2] * speed

    # Juke off the line when the cat is close.
    cat_d = dist_xy(pos, cat)
    if cat_d < THREAT_RADIUS:
        w = (THREAT_RADIUS - cat_d) / THREAT_RADIUS
        ax = (pos[0] - cat[0]) / (cat_d + 1e-6)
        ay = (pos[1] - cat[1]) / (cat_d + 1e-6)
        if t - last_flip > JUKE_FLIP:
            juke_sign *= -1.0
            last_flip = t
        px, py = -ay * juke_sign, ax * juke_sign
        tgt_vx = (1 - w) * tgt_vx + w * (px + 0.4 * ax) * DODGE_GAIN
        tgt_vy = (1 - w) * tgt_vy + w * (py + 0.4 * ay) * DODGE_GAIN

    # Accel limit, then bank into the direction of travel.
    md = ACCEL_MAX * 0.05
    vx += clamp(tgt_vx - vx, -md, md)
    vy += clamp(tgt_vy - vy, -md, md)
    vz += clamp(tgt_vz - vz, -md, md)
    yaw_cmd = 0.0
    if math.hypot(vx, vy) > 0.3:
        yaw_cmd = KYAW * wrap(math.atan2(vy, vx) - HAL.get_yaw())

    cmd_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    HAL.set_cmd_vel(vx, vy, vz, yaw_cmd)
    time.sleep(0.05)
