import gui_exercise
from console import start_console


# Create a GUI interface
gui_interface = gui_exercise.GUI()

# Spin a thread to keep the interface updated
thread_gui = gui_exercise.ThreadGUI(gui_interface)
thread_gui.start()

# Redirect the console
start_console()

def showImage(image):
    gui_interface.showImage(image)