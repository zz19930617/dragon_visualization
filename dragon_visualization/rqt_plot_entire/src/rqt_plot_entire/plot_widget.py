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
        self.setObjectName('EntirePlotWidget')


        #ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_plot_entire') , 'resource' , 'plot.ui')
        loadUi(ui_file , self)
        # Give QObjects reasonable names
        self.setObjectName('EntirePlotWidget')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
    def switch_widget_lf(self, widget):
            
        #print dir(self.layout_lf)
        self.layout_lf.addWidget(widget)
        
    def switch_widget_lb(self, widget):
            
        #print dir(self.layout_lf)
        self.layout_lb.addWidget(widget)
    
    def switch_widget_rf(self, widget):
        self.layout_rf.addWidget(widget)
        
    def switch_widget_rb(self, widget):
        self.layout_rb.addWidget(widget)
        
    def switch_widget_legs(self , widget):
        self.layout_legs.addWidget(widget)
        
    def switch_widget_lf_foot(self , widget):
        self.verticalLayout_lf.addWidget(widget)
    def switch_widget_lb_foot(self , widget):
        self.verticalLayout_lb.addWidget(widget)
    def switch_widget_rf_foot(self , widget):
        self.verticalLayout_rf.addWidget(widget)
    def switch_widget_rb_foot(self, widget):
        self.verticalLayout_rb.addWidget(widget)
        
    def switch_widget_foots(self, widget):
        self.layout_foot.addWidget(widget)