#!/usr/bin/python
#
#  Copyright (C) 1997-2015 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
#


import sys
from PyQt4 import QtCore, QtGui
from gui.GUI import MainWindow
from gui.threadGUI import ThreadGUI
from sensors.sensor import Sensor
from sensors.threadSensor import ThreadSensor
from MyAlgorithm import MyAlgorithm



# class VtkQtFrame(QtGui.QWidget):
#     def __init__(self):
#         QtGui.QWidget.__init__(self)
#         self.layout = QtGui.QVBoxLayout(self)
#         self.layout.setContentsMargins(0, 0, 0, 0)
#         self.setLayout(self.layout)
#
#         self.setSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
#         self.VTKRenderer = vtkRenderer()
#         self.VTKRenderer.SetBackground(1, 1, 1)
#         self.VTKRenderer.SetViewport( 0, 0, 1, 1)
#
#         self.VTKRenderWindow = vtkRenderWindow()
#         self.VTKRenderWindow.AddRenderer(self.VTKRenderer)
#         self.VTKRenderWindowInteractor = QVTKRenderWindowInteractor(self,rw=self.VTKRenderWindow)
#         self.layout.addWidget(self.VTKRenderWindowInteractor)
#         #
#         self.VTKCamera = vtkCamera()
#         self.VTKCamera.SetClippingRange(0.1,1000)
#         self.VTKRenderer.SetActiveCamera(self.VTKCamera)
#         #
#         self.VTKInteractorStyleSwitch = vtkInteractorStyleSwitch()
#         self.VTKInteractorStyleSwitch.SetCurrentStyleToTrackballCamera()
#         self.VTKRenderWindowInteractor.SetInteractorStyle(self.VTKInteractorStyleSwitch)
#
#         self.VTKRenderWindowInteractor.Initialize()
#         self.VTKRenderWindowInteractor.Start()
#         self.VTKRenderWindowInteractor.ReInitialize()

# class SphereActor(vtkActor):
#     def __init__(self,rad,res,r,c):
#         self.pos = numpy.array(r)
#         self.source = vtk.vtkSphereSource()
#         self.source.SetRadius(rad)
#         self.source.SetPhiResolution(res)
#         self.source.SetThetaResolution(res)
#         self.source.SetCenter(r[0],r[1],r[2])
#         self.Mapper = vtkPolyDataMapper()
#         self.Mapper.SetInput(self.source.GetOutput())
#         self.SetMapper(self.Mapper)
#         self.GetProperty().SetColor((c[0],c[1],c[2]))
#     def move_to(self,r):
#         self.pos = numpy.array(r)
#         self.source.SetCenter(r[0],r[1],r[2])
#     def set_color(self,color):
#         self.GetProperty().SetColor(color)
#     def set_rad(self,rad):
#         self.source.SetRadius(rad)
#     def get_pos(self):
#         return self.pos

if __name__ == "__main__":
    sensor = Sensor()
    algorithm=MyAlgorithm(sensor)

    app = QtGui.QApplication(sys.argv)
    myGUI = MainWindow()
    myGUI.setSensor(sensor)
    myGUI.setAlgorithm(algorithm)


    # #create our new custom VTK Qt widget
    # print "hola"
    #
    #
    # print "hola"
    #
    # for i in range(0,10):
    #     # random 3D position between 0,10
    #     r = numpy.random.rand(3)*10.0
    #     # random RGB color between 0,1
    #     c = numpy.random.rand(3)
    #     # create new sphere actor
    #     my_sphere = SphereActor(1.0,20,r,c)
    #     # add to renderer
    #     #render_widget.VTKRenderer.AddActor(my_sphere)
    #
    # # reset the camera and set anti-aliasing to 2x
    # render_widget.VTKRenderer.ResetCamera()
    # render_widget.VTKRenderWindow.SetAAFrames(2)
    #
    # # add and show
    # myGUI.gl_layout.addWidget(render_widget)


    myGUI.show()

    t1 = ThreadSensor(sensor,algorithm)
    t1.daemon=True
    t1.start()

    t2 = ThreadGUI(myGUI)
    t2.daemon=True
    t2.start()


    sys.exit(app.exec_())

