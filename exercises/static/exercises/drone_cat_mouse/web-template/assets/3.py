from GUI import GUI
from HAL import HAL
import time
from math import sin, cos
import random
# Enter sequential code!

def path4_obf(t):
	global az;return (lambda x:((lambda x:1)(x),(lambda x:0)(x),(lambda x:0)(x),(lambda x:random.uniform(-1, 1) if round(x/2%2,2)<=0.05 else az)(x)))(t)

	
# Takeoff the drone
height = 5
HAL.takeoff(height)

while True:
    # Enter iterative code!
    t = time.time()
    vx, vy, vz, yaw = path4_obf(t)
    HAL.set_cmd_vel(vx, vy, vz, yaw)
