from GUI import GUI
from HAL import HAL

while True:
    # read image from F1 camera using HAL
    image = HAL.getImage()
   
    
    # send image back to GUI 
    GUI.showImage(image)
