from cameraWidget import CameraWidget
from logoWidget import LogoWidget
import resources_rc
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QMainWindow
from form import Ui_MainWindow
from PyQt5 import QtGui
import numpy 
import time
class MainWindow(QMainWindow, Ui_MainWindow):
    updGUI=pyqtSignal()
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.logo = LogoWidget(self)
        self.logoLayout.addWidget(self.logo)
        self.logo.setVisible(True)
        self.playpause.clicked.connect(self.playpauseClicked)
        self.updGUI.connect(self.updateGUI)
        self.icon_play = QtGui.QIcon()
        self.icon_play.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.icon_stop = QtGui.QIcon()
        self.icon_stop.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)

        self.camera=CameraWidget(self)

    def set_bag(self, bag):
        self.bag = bag

    def set_pose(self , pose):
        self.pose_obj = pose

    def setAlgorithm(self, algorithm ):
        self.algorithm=algorithm

    def getAlgorithm(self):
        return self.algorithm

    def getRGBImage(self):
        return (self.bag.color_img)

    def align(self,model,data):
        """Align two trajectories using the method of Horn (closed-form) 
           and compute translational error.
        
        Input:
        model -- first trajectory (2xn1)
        data -- second trajectory (2xn2)
        
        Output:
        rot -- rotation matrix (3x3)
        trans -- translation vector (3x1)
        trans_error -- translational error per point (1xn)
        
        """
        model = model.T
        data = data.T

        if model.shape[1] > data.shape[1] :
            diff = model.shape[1] - data.shape[1]
            #remove = numpy.random.choice(range(model.shape[1]) , size = diff , replace =False)
            remove = numpy.linspace(start = 1 , stop = model.shape[1] , num = diff , endpoint=False ,dtype= int)         
            model = numpy.delete(model , remove , axis = 1)
            z=numpy.zeros((1,data.shape[1]))
            data= numpy.vstack((data,z)) 
            model= numpy.vstack((model,z))
        elif model.shape[1] < data.shape[1] :
            diff = -(model.shape[1] - data.shape[1])
            #remove = numpy.random.choice(range(data.shape[1]) , size = diff  ,replace = False)
            remove = numpy.linspace(start = 1 , stop = data.shape[1] , num = diff , endpoint=False ,dtype= int)         
            data = numpy.delete(data , remove , axis = 1)
            z=numpy.zeros((1,model.shape[1]))
            data= numpy.vstack((data,z))
            model= numpy.vstack((model,z))
        else:
            z=numpy.zeros((1,data.shape[1]))
            data= numpy.vstack((data,z)) 
            model= numpy.vstack((model,z))


        model_zerocentered = model - model.mean(1).reshape((model.mean(1).shape[0]), 1)
        data_zerocentered = data - data.mean(1).reshape((data.mean(1).shape[0]), 1)
        
        W = numpy.zeros( (3,3) )
        for column in range(model.shape[1]):
            W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
        U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
        S = numpy.matrix(numpy.identity( 3 ))
        if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
            S[2,2] = -1
        rot = U*S*Vh
        trans = (data.mean(1).reshape((data.mean(1).shape[0]), 1)) - rot * (model.mean(1).reshape((model.mean(1).shape[0]), 1))
        
        model_aligned = rot * model + trans
        alignment_error = model_aligned - data
        
        trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
            
        return trans_error

    def playpauseClicked(self):
        if self.playpause.isChecked():
            self.playpause.setText("Pause code")
            self.playpause.setIcon(self.icon_stop)
            self.algorithm.play()
        else:
            self.algorithm.stop()

            self.playpause.setText("Play code")
            self.playpause.setIcon(self.icon_play)
            pred_path = self.pose_obj.get_pred_path()
            actual_path = self.pose_obj.get_actual_path()
            trans_error = self.align(pred_path , actual_path)
            self.median.display(numpy.median(trans_error))
            self.mean.display(numpy.mean(trans_error))
            self.Std_dev.display(numpy.std(trans_error))
            self.rmse.display(numpy.sqrt(numpy.dot(trans_error,trans_error) / len(trans_error)))
    def updateGUI(self):
        #try:
        if self.bag.current_timestamp !=None :
            time_elapsed = self.bag.current_timestamp- self.bag.init_timestamp
            #except TypeError:

            #    time_elapsed = 0
            percetage_covered = time_elapsed /125.9 * 100
            self.progressBar.setValue(percetage_covered)
        else:
            time_elapsed = 0
        #self.median.display(????)
        #self.mean.display(??)
        #self.Std_dev.display(??)
        #self.rmse.display(??)s
        try:
            RT_factor = time_elapsed/(self.algorithm.diff_time)
        except (AttributeError , ZeroDivisionError):
            RT_factor = 0
        self.realtime.display(RT_factor)
        c=time.time()
        self.camera.updateImage()

        d = time.time()
    
        self.plot.update_figure(self.pose_obj.get_pred_path() , self.pose_obj.get_actual_path())
