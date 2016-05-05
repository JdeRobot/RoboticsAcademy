from sensors import sensor
import numpy as np
import cv2

class MyAlgorithm():

    def __init__(self, sensor, grid):
        self.sensor = sensor
        self.grid = grid
        sensor.getPathSig.connect(self.generatePath)

    """ Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Generate Path button. 
        Call to grid.setPath(path) mathod for setting the path. """
    def generatePath(self):
        print "LOOKING FOR SHORTER PATH"
        mapIm = self.grid.getMap()      
        dest = self.grid.getDestiny()   
        gridPos = self.grid.getPose()

        #TODO

        #Represent the Gradient Field in a window using cv2.imshow
        self.grid.showGrid()

    """ Write in this mehtod the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """
    def execute(self):
        # Add your code here
        print "GOING TO DESTINATION"
