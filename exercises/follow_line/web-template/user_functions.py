from __future__ import print_function

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
class HALFunctions(UserFunctions):
    def __init__(self, hal_pipe):
        # Initialize the pipe
        super(HALFunctions, self).__init__(hal_pipe)

    # Get image function
    def getImage(self):
        execution_string = "self.hal.getImage()"
        self.send(execution_string)
        ret_obj = self.recv()

        return ret_obj

    # Send velocity function
    def sendV(self, velocity):
        execution_string = "self.hal.motors.sendV(%f)" % velocity
        self.send(execution_string)
        ret_obj = self.recv()

        return ret_obj

    # Send angular velocity function
    def sendW(self, angular):
        execution_string = "self.hal.motors.sendW(%f)" % angular
        self.send(execution_string)
        ret_obj = self.recv()

        return ret_obj

# Define GUI functions
class GUIFunctions(UserFunctions):
    def __init__(self, gui_pipe):
        super(GUIFunctions, self).__init__(gui_pipe)

    # Show image function
    def showImage(self, image):
        execution_string = "self.gui.showImage()"
        self.send(execution_string)
        self.send(image)
        ret_obj = self.recv()

        return ret_obj