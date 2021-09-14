from GUI import GUI
from HAL import HAL
import time
from math import sin, cos
import random
# Enter sequential code!

def path1_obf(t):
	return (lambda x:((lambda x:3 if round(x/2)%2==0 else 0)(x),(lambda x:0)(x),(lambda x:0)(x),(lambda x:0)(x)))(t)
	
# Takeoff the drone
height = 5
HAL.takeoff(height)

while True:
    # Enter iterative code!
    t = time.time()
    vx, vy, vz, yaw = path1_obf(t)
    HAL.set_cmd_vel(vx, vy, vz, yaw)
