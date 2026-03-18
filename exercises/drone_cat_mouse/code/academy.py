# Enter sequential code!
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from GUI import GUI
from HAL import HAL

while True:
    # Enter iterative code!
    img = HAL.get_ventral_image()
    GUI.showImage(img)
