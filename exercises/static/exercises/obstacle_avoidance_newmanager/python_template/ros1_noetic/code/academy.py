# Enter sequential code!
from exercises.static.exercises.obstacle_avoidance_newmanager.python_template.ros1_noetic.GUI import GUI
from exercises.static.exercises.obstacle_avoidance_newmanager.python_template.ros1_noetic.HAL import HAL

while True:
    # Enter iterative code!
    img = HAL.getImage()
    GUI.showImage(img)