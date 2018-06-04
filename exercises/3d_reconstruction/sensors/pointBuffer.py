import sys, traceback, Ice
import easyiceconfig as EasyIce
import comm, config
import jderobot
import numpy as np
import threading
import sensor
import cv2
from random import uniform



bufferpoints = []
bufferline = []
refresh = False

class PointI(jderobot.Visualization):
    def getSegment(self, current=None):
        rgblinelist = jderobot.bufferSegments()
	rgblinelist.buffer = []
	rgblinelist.refresh = refresh
	for i in bufferline[:]:
            rgblinelist.buffer.append(i)
            index = bufferline.index(i)
            del bufferline[index]
        return rgblinelist

    def drawPoint(self,point, color, current=None):
        print point

    def getPoints(self, current=None):
        rgbpointlist = jderobot.bufferPoints()
	rgbpointlist.buffer = []
	rgbpointlist.refresh = refresh
        for i in bufferpoints[:]:
            rgbpointlist.buffer.append(i)
            index = bufferpoints.index(i)
            del bufferpoints[index]
        return rgbpointlist

    def clearAll(self, current=None):
        print "Clear All"

def getbufferSegment (seg,color,plane):
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

try:
    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg, '3DReconstruction')
    endpoint = jdrc.getConfig().getProperty("3DReconstruction.Viewer.Endpoint")
    proxy = jdrc.getConfig().getProperty("3DReconstruction.Viewer.Proxy")
    refresh = jdrc.getConfig().getProperty("3DReconstruction.Viewer.Refresh")
    id = Ice.InitializationData()
    ic = Ice.initialize(None, id)
    adapter = ic.createObjectAdapterWithEndpoints(proxy, endpoint)
    object = PointI()
    adapter.add(object, ic.stringToIdentity("3DViz"))
    adapter.activate()
except KeyboardInterrupt:
	del(ic)
	sys.exit()
