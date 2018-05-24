import sys, traceback, Ice
import easyiceconfig as EasyIce
import comm, config
import jderobot
import numpy as np
import threading
import sensor
import cv2

bufferpoints = []
bufferline = []
colorline = ""

class PointI(jderobot.Visualization):
    def getSegment(self, current=None):
        rgblinelist = []
        rgblinelist = bufferline
        return rgblinelist
        
    def drawPoint(self,point, color, current=None):
        print point

    def getPoints(self, current=None):
        rgbpointlist = []
        for i in bufferpoints[:]:
            rgbpointlist.append(i)
            index = bufferpoints.index(i)
            del bufferpoints[index]
        return rgbpointlist

    def clearAll(self, current=None):
        print "Clear All"


try:
    endpoint = "default -h localhost -p 9957:ws -h localhost -p 11000"
    id = Ice.InitializationData()
    ic = Ice.initialize(None, id)
    adapter = ic.createObjectAdapterWithEndpoints("3DViewerA", endpoint)
    object = PointI()
    adapter.add(object, ic.stringToIdentity("3DViewer"))
    adapter.activate()
    #ic.waitForShutdown()
except KeyboardInterrupt:
	del(ic)
	sys.exit()

def getbufferSegment (seg,color):
    rgbsegment = jderobot.RGBSegment()
    rgbsegment.seg = seg
    rgbsegment.c = color
    bufferline.append(rgbsegment)

def getbufferPoint(point, color):
    rgbpoint = jderobot.RGBPoint()
    rgbpoint.x = point.x
    rgbpoint.y = point.y
    rgbpoint.z = point.z
    rgbpoint.r = color.r
    rgbpoint.g = color.g
    rgbpoint.b = color.b
    bufferpoints.append(rgbpoint)
