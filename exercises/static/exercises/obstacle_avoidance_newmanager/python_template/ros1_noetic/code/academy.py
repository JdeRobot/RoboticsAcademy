# Enter sequential code!
from exercises.static.exercises.obstacle_avoidance_newmanager.python_template.ros1_noetic.gui import GUI
from exercises.static.exercises.obstacle_avoidance_newmanager.python_template.ros1_noetic.hal import HAL

while True:
    # Enter iterative code!
    img = HAL.getImage()
    GUI.showImage(img)