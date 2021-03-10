from __future__ import print_function
from shared.image import SharedImage

# Base class for user functions
class UserFunctions(object):
    # Initialize pipe
    def __init__(self, pipe):
        self.pipe = pipe

    # Close pipe
    def close(self):
        self.pipe.close()

    # Send message through pipe
    def send(self, command):
        self.pipe.send(command)

    # Receive a message through pipe
    def recv(self):
        message = self.pipe.recv()
        return message

# Define Console functions
class ConsoleFunctions(UserFunctions):
    def __init__(self, console_pipe):
        # Initialize the pipe
        super(ConsoleFunctions, self).__init__(console_pipe)

    # Print function
    def print(self, message):
        execution_string = "self.console.print(\"%s\")" % message
        self.send(execution_string)
        ret_obj = self.recv()

        return ret_obj

# Define HAL functions
class HALFunctions:
    def __init__(self):
        # Initialize image variable
        self.shared_image = SharedImage("halimage")

    # Get image function
    def getImage(self):
        image = self.shared_image.get()
        return image

    # Send velocity function
    def sendV(self, velocity):
        pass

    # Send angular velocity function
    def sendW(self, angular):
        pass

# Define GUI functions
class GUIFunctions:
    def __init__(self):
        # Initialize image variable
        self.shared_image = SharedImage("guiimage")

    # Show image function
    def showImage(self, image):
        self.shared_image.add(image)