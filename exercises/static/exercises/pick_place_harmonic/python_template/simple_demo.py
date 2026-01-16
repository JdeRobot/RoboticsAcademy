#!/usr/bin/env python
"""
SIMPLE WORKING DEMO - Pick and Place Harmonic
This version uses simple, reachable positions that work reliably
"""

from hal_api.HAL import get_hal

# Get HAL interface
hal = get_hal()

print("="*50)
print("Pick and Place Harmonic - Simple Demo")
print("="*50)

# Open gripper to start
print("\n[1/7] Opening gripper...")
hal.open_gripper()
hal.sleep(2)

# Move to safe home position
print("\n[2/7] Moving to home position...")
hal.move_to_pose(x=0.0, y=-0.3, z=0.3)
hal.sleep(2)

# Move forward (simulating above object)
print("\n[3/7] Moving above 'object'...")
hal.move_to_pose(x=0.4, y=0.0, z=0.25)
hal.sleep(2)

# Move down slightly (simulating pick)
print("\n[4/7] Moving down to 'pick'...")
hal.move_to_pose(x=0.4, y=0.0, z=0.18)
hal.sleep(2)

# Close gripper (grasp 50mm object)
print("\n[5/7] Grasping object...")
hal.grasp(width=0.050)
hal.sleep(3)

# Lift up
print("\n[6/7] Lifting object...")
hal.move_to_pose(x=0.4, y=0.0, z=0.25)
hal.sleep(2)

# Move to different location (simulating drop)
print("\n[7/7] Moving to drop location...")
hal.move_to_pose(x=0.0, y=0.3, z=0.25)
hal.sleep(2)

# Release
print("\n[8/8] Releasing object...")
hal.open_gripper()
hal.sleep(2)

print("\n" + "="*50)
print("Demo complete!")
print("="*50)
