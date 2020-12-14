from GUI import GUI
from HAL import HAL
# Enter sequential code!
import cv2
import numpy as np


x_middle_left_above = 0
desviation_left = 0


while True:
    # Enter iterative code!
    image = HAL.getImage()
    
    # Convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    low_thresh = np.array([0, 200, 0])
    upper_thresh = np.array([140, 245, 255])
    
    # Filtering image
    image_HSV_filtered = cv2.inRange(hsv, low_thresh, upper_thresh)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31))
    image_HSV_filtered = cv2.morphologyEx(image_HSV_filtered, cv2.MORPH_CLOSE, kernel)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    image_HSV_filtered = cv2.morphologyEx(image_HSV_filtered, cv2.MORPH_OPEN, kernel)
    
    GUI.showImage(image)
    
    # Shape gives us the number of rows and columns of an image
    size = image_HSV_filtered.shape
    rows = size[0]
    columns = size[1]
    
    # We look for the position on the x axis of the pixels that have value 1 in different positions and
    position_x_down = np.where(image_HSV_filtered[350, :])
    position_x_above = np.where(image_HSV_filtered[260, :])
    position_x_middle = np.where(image_HSV_filtered [310, :])
    
    
    if (len(position_x_down[0]) > 1):
        x_middle_left_down = (position_x_down[0][0] + position_x_down[0][len(position_x_down[0]) - 1]) / 2
        not_found_down = False
    else:
        # The center of the line is in position 320
        x_middle_left_down = 320
        not_found_down = True
    
    if (len(position_x_middle[0]) > 1):
        x_middle_left_middle = (position_x_middle[0][0] + position_x_middle[0][len(position_x_middle[0]) - 1]) / 2
        not_found_middle = False
    else:
        # The center of the line is in position 320
        x_middle_left_middle = 320
        not_found_middle = True
        
    # We look if white pixels of the row above are located
    if (len(position_x_above[0]) > 1):
        x_middle_left_above = (position_x_above[0][0] + position_x_above[0][len(position_x_above[0]) - 1]) / 2
        # We look at the deviation from the central position. The center of the line is in position 326
        desviation = x_middle_left_above - 320

        # If the row below has been lost we have a different case, which we treat as an exception
        if not_found_down == True:
            dif = x_middle_left_middle - x_middle_left_above

            if (abs(dif) < 80):
                rotation = -(0.03 * desviation)
                HAL.motors.sendW(rotation)
            elif (abs(dif) < 130):
                rotation = -(0.073 * desviation)
    
            elif (abs(dif) < 190):
                rotation = -(0.082 * desviation)
    
            else:
                rotation = -(0.090 * desviation)
    
            speed = 1
            HAL.motors.sendW(rotation)
            HAL.motors.sendV(speed)
        else:
            # We check is formula 1 is in curve or straight
            dif = x_middle_left_down - x_middle_left_above
            x = float(((-dif) * (310 - 350))) / float(260-350) + x_middle_left_down

            if abs(x - x_middle_left_middle) < 2:
               # cv2.putText(imageRight_copy.data, 'Straight', (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 0), 2)
                if (abs(dif) < 35):
                    rotation = -(0.0054 * desviation)
                    speed = 4
        
                elif (abs(dif) < 90):
                    rotation = -(0.0052 * desviation)
                    speed = 3
        
                else:
                    rotation = -(0.049 * desviation)
                    speed = 2
        
                HAL.motors.sendW(rotation)
                HAL.motors.sendV(speed)
            else:
                #cv2.putText(imageRight_copy.data, 'Curve', (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 0), 2)
                if (abs(dif) < 50):
                    rotation = -(0.1 * desviation)
        
                if (abs(dif) < 80):
                    rotation = -(0.0078 * desviation)
        
                elif (abs(dif) < 130):
                    rotation = -(0.0081 * desviation)
        
                elif (abs(dif) < 190):
                    rotation = -(0.0082 * desviation)
        
                else:
                    rotation = -(0.0090 * desviation)
        
                speed = 1
                HAL.motors.sendW(rotation)
                HAL.motors.sendV(speed)

        
        console.print(desviation)
        console.print("dif: " + str(dif))
        
        # We update the desviation
        desviation_left = desviation
    else:
        # If the formula 1 leaves the red line, the line is searched
        if x_middle_left_above > (columns/2):
            HAL.motors.sendW(-1)
            rotation = -1
        else:
            HAL.motors.sendW(1)
            rotation = 1
        HAL.motors.sendV(-0.6)
        speed = -0.6
        
    console.print("speed: " + str(speed))
    console.print("rotation: " + str(rotation))
  