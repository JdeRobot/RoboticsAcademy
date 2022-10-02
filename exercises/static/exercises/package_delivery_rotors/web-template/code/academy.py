# Enter sequential code!
from GUI import GUI
from HAL import HAL

while True:
    # Enter iterative code!
    img = HAL.get_ventral_image()
    GUI.showImage(img)