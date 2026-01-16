import HAL
import WebGUI
import Frequency

# ========== Sequential Code - Runs Once ==========

# Setup collision scene (table, conveyor, bins)
HAL.setup_workspace()
HAL.sleep(3)  # Let MoveIt process collision objects

# TODO: Write your pick and place solution here!
# Example:
# HAL.open_gripper()
# HAL.move_to_pose(x=0.0, y=-0.4, z=0.4)  # Move to home position
# HAL.move_to_pose(x=0.6, y=-0.3, z=0.285)  # Move above red box
# HAL.move_cartesian(x=0.6, y=-0.3, z=0.265)  # Lower to red box (Cartesian for precision)
# HAL.grasp(width=0.055)  # Grasp red box
# ...

# ========== Iterative Code - Runs Continuously ==========
while True:
    # TODO: Add iterative logic here if needed
    # This loop runs continuously
    
    Frequency.tick()  # Maintain loop frequency
