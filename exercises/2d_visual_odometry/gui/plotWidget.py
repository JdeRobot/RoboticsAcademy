import pyqtgraph as pg
from PyQt5 import QtWidgets

class PlotWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)   # Inherit from QWidget
        self.winparent = parent
        self.vbl = QtWidgets.QVBoxLayout()         # Set box for plotting
        #pg.setConfigOption('background', 'w')
        self.canvas = pg.GraphicsLayoutWidget()
        self.vbl.addWidget(self.canvas)
        self.setLayout(self.vbl)
        

        #  line plot
        self.lineplot = self.canvas.addPlot()
        self.lineplot.addLegend()
        self.pred_plot = self.lineplot.plot(name='predicted position')
        self.actual_plot = self.lineplot.plot(name='actual position')

    def update_figure(self , pred , actual):
        if len(pred) > 0 and len(actual)>0:
            self.pred_plot.setData(pred[:,0],pred[:,1] , pen = pg.mkPen('r', width=2) ,antialias =True)
            self.actual_plot.setData(actual[:,0],actual[:,1] , pen =pg.mkPen('b', width=2) ,antialias =True) 
