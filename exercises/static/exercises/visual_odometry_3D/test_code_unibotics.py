from GUI import GUI
from HAL import HAL
# Enter sequential code!

sum = 0
while True:
# Enter iterative code!
    sum = sum + 1
    print("hola", sum)
    
    img = HAL.get_image("left")
    GUI.show_image(img)

    true_t = HAL.get_current_groundtruth_position()
    HAL.set_estimated_position(true_t[0], true_t[1], true_t[2])

    true_euler = HAL.get_true_euler_angles_corrected()
    HAL.set_estimated_euler_angles(true_euler[0], true_euler[1], true_euler[2])
    
    HAL.advance()
