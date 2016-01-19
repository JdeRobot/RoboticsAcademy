from sensors import sensor

H_SIZE = 100
D_UP = 1
D_DOWN = 2
D_RIGHT = 3
D_LEFT = 4

class MyAlgorithm():
    def __init__(self, sensor):
      self.sensor = sensor

    def execute(self):
        # Add your code here
        tmp = self.sensor.getNavdata()
        if tmp is not None:
            print "State: " +str(tmp.state)
            print "Altitude: " +str(tmp.altd)
            print "Vehicle: " +str(tmp.vehicle)
            print "Battery %: " +str(tmp.batteryPercent)


