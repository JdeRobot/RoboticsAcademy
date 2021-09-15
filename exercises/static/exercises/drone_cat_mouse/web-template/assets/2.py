from GUI import GUI
from HAL import HAL
import time
from math import sin, cos
import random
# Enter sequential code!

def path3_obf(t):
	return (lambda x:((lambda x:1)(x),(lambda x:sin(x/2)*1)(x),(lambda x:cos(x/2)*1)(x),(lambda x:0)(x)))(t)

	
# Takeoff the drone
height = 5
HAL.takeoff(height)

while True:
    # Enter iterative code!
    t = time.time()
    vx, vy, vz, yaw = path3_obf(t)
    HAL.set_cmd_vel(vx, vy, vz, yaw)
