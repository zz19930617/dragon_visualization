import argparse

from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug
from rqt_gui_py.plugin import Plugin

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget

from rqt_py_common.ini_helper import pack, unpack

from .plot_widget import PlotWidget

from .data_plot import DataPlot

class Plot(PlotWidget):
    
    def __init__(self ):
        super(Plot , self).__init__()
        self.setObjectName('plot')
        
        self._data_plot = DataPlot(self)
        
        #set parameters of data_plot 
        self._data_plot.set_autoscale(x=False)
        self._data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND | DataPlot.SCALE_VISIBLE)
        self._data_plot.set_xlim([0 , 10.0])
        
        #self._widget.switch_data_plot_widget(self._data_plot)
        self.switch_data_plot_widget(self._data_plot)
