from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtOpenGL

import OpenGL.GL as gl
from OpenGL import GLU

from OpenGL.arrays import vbo
import numpy as np

import sys

class GroundGraphics(object):
    
    def __init__(self, length, width):
        self.len = length
        self.w = width
        self.res = 15
        self.n_sq = self.res ** 2
        self.n_vert = 6 * self.n_sq

        self.vx = np.linspace(-0.5 * self.len, 0.5 * self.len, self.res + 1)
        self.vy = np.linspace(-0.5 * self.w, 0.5 * self.w, self.res + 1)
        self.vz = np.zeros((self.res + 1, self.res + 1))

        self.vert = np.zeros((self.n_vert, 3))

        sq_ind = 0
        for i in range(self.res):
            for j in range(self.res):
                self.vert[6 * sq_ind, :] = np.array([self.vx[i], self.vy[j], self.vz[i, j]])
                self.vert[6 * sq_ind + 1, :] = np.array([self.vx[i+1], self.vy[j+1], self.vz[i+1, j+1]])
                self.vert[6 * sq_ind + 2, :] = np.array([self.vx[i], self.vy[j+1], self.vz[i, j+1]])

                self.vert[6 * sq_ind + 3, :] = np.array([self.vx[i], self.vy[j], self.vz[i, j]])
                self.vert[6 * sq_ind + 4, :] = np.array([self.vx[i+1], self.vy[j], self.vz[i+1, j]])
                self.vert[6 * sq_ind + 5, :] = np.array([self.vx[i+1], self.vy[j+1], self.vz[i+1, j+1]])

                sq_ind += 1

        self.vert_stride = 12
        self.vert_vbo = vbo.VBO(np.reshape(self.vert, (1, -1), order='C').astype(np.float32))


    def render(self):
        gl.glPushMatrix()

        try:
            self.vert_vbo.bind()

            gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
            gl.glVertexPointer(3, gl.GL_FLOAT, self.vert_stride, self.vert_vbo)
            
            gl.glColor3f(1.0, 1.0, 1.0)
            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)

            gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.n_vert)

            gl.glColor3f(0.5, 0.5, 0.5)
            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)

            gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.n_vert)

        except Exception as e:
            print(e)

        finally:
            self.vert_vbo.unbind()
            gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

            gl.glPopMatrix()


class PointGraphic(object):
    
    def __init__(self):
        self.sphere = GLU.gluNewQuadric()
        GLU.gluQuadricNormals(self.sphere, GLU.GLU_SMOOTH)
        GLU.gluQuadricTexture(self.sphere, gl.GL_TRUE)

    def render(self, start, color, radius=0.03):
        gl.glPushMatrix()
	gl.glColor3f(*color)
        gl.glTranslatef(*start)
        GLU.gluSphere(self.sphere, radius, 10, 10)
        gl.glPopMatrix()

class GLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        self.parent = parent
        QtOpenGL.QGLWidget.__init__(self, parent)

	self.points = []
	self.setMouseTracking(False)

    def initializeGL(self):
        self.qglClearColor(QtGui.QColor(100, 100, 100))
        gl.glEnable(gl.GL_DEPTH_TEST)
        self.initGeometry()
        self.ground_graphics = GroundGraphics(length=15.0, width=15.0)

        self.eye_r = 20.0
        self.eye_th = 1.0
        self.eye_phi = 1.0
        self.center_pos = np.array([0.0, 0.0, 0.0])

        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        self.point_graphics = PointGraphic()
        

    def resizeGL(self, width, height):
        gl.glViewport(0, 0, width, height)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = width / float(height)

        GLU.gluPerspective(45.0, aspect, 1.0, 100.0)
        gl.glMatrixMode(gl.GL_MODELVIEW)

    def initGeometry(self):
        pass
    
    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        
        self.ground_graphics.render()
        self.accumulate_points()
	self.plot_point()
        #self.origin_axes_graphics.render(np.identity(3), np.zeros(3))
        #self.point_graphics.render(np.array([1.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0]))
        #self.point_graphics.render(np.array([0.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))

    
    def accumulate_points(self):
    	point = self.parent.getPlot().getPoint()
    	if (point not in self.points) and len(point) > 0:
    		self.points.append(point)
    
    
    def plot_point(self):
	for point in self.points:
    		self.point_graphics.render(np.array(point[:3]), np.array(point[3:]))
    
    def update_view(self):
        self.eye_pos = np.array([self.eye_r*np.sin(self.eye_phi)*np.cos(self.eye_th),
                                 self.eye_r*np.sin(self.eye_phi)*np.sin(self.eye_th),
                                 self.eye_r*np.cos(self.eye_phi)])

        up_vec = np.array([0.0, 0.0, 1.0])
        gl.glLoadIdentity()
        GLU.gluLookAt(*np.concatenate((self.eye_pos, self.center_pos, up_vec)))

    
    def reset_view(self):
    	self.eye_r = 20.0
    	self.eye_th = 1.0
    	self.eye_phi = 1.0
    	self.center_pos = np.array([0.0, 0.0, 0.0])
    	self.update_view()

    def mousePressEvent(self, event):
    	if event.button() == QtCore.Qt.LeftButton:
    		self.mem_pos = [event.pos().x(), event.pos().y()]
    		self.mem_param = [self.eye_th, self.eye_phi]
    		self.setMouseTracking(True)
    		
    def mouseReleaseEvent(self, event):
    	self.setMouseTracking(False)
    		
    
    def mouseMoveEvent(self, event):
    	self.eye_th = self.mem_param[0] + 0.01 * (self.mem_pos[0] - event.x())
    	self.eye_phi = self.mem_param[1] + 0.01 * (self.mem_pos[1] - event.y())
    	
    	self.update_view()
    	
    def wheelEvent(self, event):
    	self.eye_r += event.angleDelta().y() * 0.01
    	self.update_view()
    
    def keyPressEvent(self, event):

        if type(event) == QtGui.QKeyEvent:
            if event.key() == QtCore.Qt.Key_W:
                self.eye_r -= 0.5
                self.update_view()

            elif event.key() == QtCore.Qt.Key_S:
                self.eye_r += 0.5
                self.update_view()

            elif event.key() == QtCore.Qt.Key_Down:
                self.eye_phi -= 0.05
                self.update_view()

            elif event.key() == QtCore.Qt.Key_Up:
                self.eye_phi += 0.05
                self.update_view()

            elif event.key() == QtCore.Qt.Key_Right:
                self.eye_th += 0.05
                self.update_view()

            elif event.key() == QtCore.Qt.Key_Left:
                self.eye_th -= 0.05
                self.update_view()
