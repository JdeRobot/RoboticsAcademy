from sensors import sensor

AREA = [[7.0, -4.0], [7.0, 3.0], [-1.0, 7.0], [-7.0, 0.5], [-0.5, -7.0]]
CASCPATH = "haarcascade_frontalface_default.xml"

class MyAlgorithm():
  def __init__(self, sensor):
    self.sensor = sensor

  def execute(self):
    #Add your code here
    tmp = self.sensor.getNavdata()
    if tmp is not None:
        print "State: " +str(tmp.state)
        print "Altitude: " +str(tmp.altd)
        print "Vehicle: " +str(tmp.vehicle)
        print "Battery %: " +str(tmp.batteryPercent)



