"""Palletizing — reference solution.

A conveyor feeds boxes one at a time to a fixed pickup point; the robot picks each
box with a suction cup and stacks them into an ordered 2x2x2 grid on the pallet.

The feeder handshake is wrapped by the HAL, so this code never touches ROS:
  - HAL.WaitForBox()   blocks until a box is stopped at the pickup point, returns its name
  - HAL.BoxDone(name)  reports the box is stacked, releasing the next one

All motion targets are in the robot BASE frame. The robot is mounted 0.9 m up in
the Gazebo world, so a world height z maps to base height (z - BASE_MOUNT_Z).
"""

import HAL_Harmonic as HAL

# --- Geometry (Gazebo world coordinates unless noted) -----------------------
BASE_MOUNT_Z = 0.9        # robot base height in the world; base_z = world_z - this

# The suction cup is not coaxial with tool0: it lands CUP_TOOL0_DX ahead of tool0
# in world-X. Aim tool0 that much short so the cup centres on the box.
CUP_TOOL0_DX = 0.05
PICK_WORLD_X = 1.0 - CUP_TOOL0_DX   # box centre stops at world X=1.0
PICK_WORLD_Y = 0.0

BOX_HALF_HEIGHT = 0.10
PICK_BOX_TOP_WORLD_Z = 1.20   # box top surface at the pickup point
CUP_REACH = 0.075             # tool0 sits this far above the cup-face contact (calibrated)

TRANSIT_Z = 0.60          # base-frame cruise height; clears the belt and the tallest stack
CUP_DOWN_YPR = [0.0, 179.0, 0.0]   # cup faces straight down (roll, pitch, yaw in deg)

# --- Pallet grid: 2 cols x 2 rows x 2 layers, aligned stack ------------------
GRID_COLS = 2
GRID_ROWS = 2
GRID_LAYERS = 2
GRID_CENTER_WORLD_X = 0.0
GRID_CENTER_WORLD_Y = -0.88   # must match the target_pallet pose in the world
GRID_BASE_WORLD_Z = 0.40      # bottom-layer box centre (deck top 0.30 + box half 0.10)
PITCH_X = 0.42                # column spacing (box 0.40 + 0.02 gap)
PITCH_Y = 0.32                # row spacing (box 0.30 + 0.02 gap)
PITCH_Z = 0.20                # layer spacing (box height)
PLACE_YAW = 0.0

# --- Motion tuning ----------------------------------------------------------
SPEED = 0.7        # fraction of max [0, 1]
SETTLE = 1.0       # pause after each move (s); do not lower, or execution gets rejected
GRIP_PAUSE = 0.5   # pause for the suction to attach/detach (s)


def world_to_base_z(world_z):
    """Convert a Gazebo world height to the robot base-frame height."""
    return world_z - BASE_MOUNT_Z


def grid_cell(index):
    """Map a box index to its centre (x, y, z), yaw, and layer/row/col in world coords."""
    per_layer = GRID_COLS * GRID_ROWS
    layer = index // per_layer
    within = index % per_layer
    row = within // GRID_COLS
    col = within % GRID_COLS

    x = GRID_CENTER_WORLD_X + (col - (GRID_COLS - 1) / 2.0) * PITCH_X
    y = GRID_CENTER_WORLD_Y + (row - (GRID_ROWS - 1) / 2.0) * PITCH_Y
    z = GRID_BASE_WORLD_Z + layer * PITCH_Z
    return x, y, z, PLACE_YAW, layer, row, col


def pick():
    """Descend onto the box at the pickup point, grip it, and lift to cruise height."""
    bx = PICK_WORLD_X
    by = PICK_WORLD_Y
    grip_z = world_to_base_z(PICK_BOX_TOP_WORLD_Z + CUP_REACH)

    HAL.MoveJoint([bx, by, TRANSIT_Z], CUP_DOWN_YPR, SPEED, SETTLE)   # over the box
    HAL.MoveLinear([bx, by, grip_z], CUP_DOWN_YPR, SPEED, SETTLE)     # down onto it
    HAL.SuctionSet(True, GRIP_PAUSE)                                  # grip
    HAL.MoveLinear([bx, by, TRANSIT_Z], CUP_DOWN_YPR, SPEED, SETTLE)  # lift clear


def place(index):
    """Carry the held box to grid cell `index` and release it onto the stack."""
    wx, wy, wz, yaw, layer, row, col = grid_cell(index)
    place_z = world_to_base_z(wz + BOX_HALF_HEIGHT + CUP_REACH)
    place_ypr = [CUP_DOWN_YPR[0], CUP_DOWN_YPR[1], yaw]
    tx = wx - CUP_TOOL0_DX   # cup offset, as at pick, so the box centres on the cell

    HAL.MoveJoint([tx, wy, TRANSIT_Z], place_ypr, SPEED, SETTLE)   # over the cell
    HAL.MoveLinear([tx, wy, place_z], place_ypr, SPEED, SETTLE)    # down onto the stack
    HAL.SuctionSet(False, GRIP_PAUSE)                             # release
    HAL.MoveLinear([tx, wy, TRANSIT_Z], place_ypr, SPEED, SETTLE)  # retreat up
    print(f"placed box {index} at grid L{layer} R{row} C{col} (yaw {yaw:.0f})")


def main():
    home_joints = [0.0, -90.0, 0.0, -90.0, 0.0, 0.0]
    HAL.MoveAbsJ(home_joints, SPEED, SETTLE)

    total = GRID_COLS * GRID_ROWS * GRID_LAYERS
    for index in range(total):
        name = HAL.WaitForBox()
        if name is None:
            break
        print(f"box {name} ready -> placing as #{index}")

        pick()
        HAL.BoxDone(name)   # box is lifted clear -> free the belt to feed the next one
        place(index)

    print("palletizing complete")


if __name__ == "__main__":
    main()
