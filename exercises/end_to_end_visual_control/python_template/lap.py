from datetime import datetime
import signal


class Lap:
    def __init__(self, pose3d):
        self.pose3d = pose3d
        self.reset()

        def signal_handler(sign, frame):
            self.start_time = datetime.now()

        signal.signal(signal.SIGCONT, signal_handler)

    # Function to check for threshold
    # And incrementing the running time
    def check_threshold(self):

        # Running condition to calculate the current time
        # Time calculated by adding increments from each iteration
        if self.start_time != 0 and self.lap_rest == False:
            if self.lap_time == 0:
                self.lap_time = datetime.now() - self.start_time
            else:
                self.lap_time += datetime.now() - self.start_time

            self.start_time = datetime.now()

        # Condition when the time starts running
        if self.start_time == 0 and self.lap_rest == True:
            self.start_time = datetime.now()
            self.lap_rest = False

        return self.lap_time

    # Function to return lap time
    def return_lap_time(self):
        return self.lap_time

    # Function to reset
    def reset(self):
        # Reset the initial variables
        self.start_time = 0
        self.lap_time = 0

        self.lap_rest = True
        self.buffer = False
