from sensors import sensor


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
