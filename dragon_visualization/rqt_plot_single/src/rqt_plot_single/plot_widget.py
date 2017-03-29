import os
import rospy
import rospkg

from qt_gui.plugin import Plugin

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class PlotWidget(QWidget):

    def __init__(self):
        super(PlotWidget, self).__init__()
        # Give QObjects reasonable names
        self.setObjectName('SinglePlotWidget')


        #ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_plot_single') , 'resource' , 'plot.ui')
        loadUi(ui_file , self)
        # Give QObjects reasonable names
        self.setObjectName('SinglePlotWidget')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
    def switch_widget_lf(self, widget):
            
        #print dir(self.layout_lf)
        self.layout_hip_pos.addWidget(widget)
        
    def switch_widget_lb(self, widget):
            
        #print dir(self.layout_lf)
        self.layout_knee_pos.addWidget(widget)
    
    def switch_widget_rf(self, widget):
        self.layout_hip_vel.addWidget(widget)
        
    def switch_widget_rb(self, widget):
        self.layout_knee_vel.addWidget(widget)