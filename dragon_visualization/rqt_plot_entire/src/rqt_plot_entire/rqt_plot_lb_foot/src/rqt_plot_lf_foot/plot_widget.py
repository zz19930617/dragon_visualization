import os
import rospkg
import threading
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import rospy

import numpy as np
from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

MOTOR_TOPIC_NAME = "/dragon/test3d"
LEG_NAME = ['L-F' , 'L-B' ,'R-F' , 'R-B']

class RosPlotException(Exception):
    pass

class PlotWidget(QWidget):
    _redraw_interval = 100
    def __init__(self):
        super(PlotWidget , self).__init__()
        self.setObjectName('PlotWidget')
        
        #ui
        rp = rospkg.RosPack()
        ui_file = os.path.join('/home/robot/catkin_ws/src/dragon_visualization/rqt_plot_entire/src/rqt_plot_entire/rqt_plot_lb_foot' , 'resource' , 'plot.ui')
        loadUi(ui_file , self)
        
        #subscribe
        self._motor_topic_name = MOTOR_TOPIC_NAME
        try:
            self._motor_subscriber = rospy.Subscriber(self._motor_topic_name , Float64MultiArray , self._motor_cb)
        except ValueError,e:
            rospy.logerr('rqt_plot: error connect topic (%s)'%e)
            
        #timer
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        
        #variable
        self._joint_data = LEG_NAME
        self.curve = {}
        for i in range(len(self._joint_data)):
            self.curve[self._joint_data[i]] = {'topic_name':self._joint_data[i] , 'buff_x':[] , 'buff_y':[] , 'buff_z':[] , 'buff_x_temp':[] , 'buff_y_temp' : [] , 'buff_z_temp': [], 'enable' : False}
        for key in self._joint_data:
            if self.curve[key]['topic_name'] == 'L-B':
                self.curve[key]['enable'] = True
        self.lock = threading.Lock()
        self.start_time = rospy.get_time()
        
        self.data_plot = None
        self.error = None
        
        
    def switch_data_plot_widget(self , data_plot):
        self.enable_timer(enabled = False)
        
        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()
        
        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(enabled = True)
        
        #data_x , data_y = self.next()
        self.next()
        for key in self.curve:
            if self.curve[key]['enable']:
            #self.data_plot.add_curve(self._topic_name , self._topic_name , data_x, data_y)
                self.data_plot.add_curve(self.curve[key]['topic_name'] , self.curve[key]['topic_name'] , self.curve[key]['buff_x_temp'] , self.curve[key]['buff_y_temp'] , self.curve[key]['buff_z_temp'])

        self.data_plot.redraw()
        
    def _motor_cb(self , msg):
        try:
            self.lock.acquire()
            try:
                for key in self.curve:
                    if self.curve[key]['enable']:
                        self.curve[key]['buff_x'].append(msg.data[0])
                        self.curve[key]['buff_y'].append(msg.data[1])
                        self.curve[key]['buff_z'].append(msg.data[2])
            except AttributeError as e:
                self.error = RosPlotException("invalid topic data")
        
        finally:
            self.lock.release()
    
    def next(self):
        #return next data of topic like [xdata] [ydata]
        if self.error:
            raise self.error
        try:
            self.lock.acquire()
            for key in self.curve:
                self.curve[key]['buff_x_temp'] = self.curve[key]['buff_x']
                self.curve[key]['buff_y_temp'] = self.curve[key]['buff_y']
                self.curve[key]['buff_z_temp'] = self.curve[key]['buff_z']
                self.curve[key]['buff_x'] = []
                self.curve[key]['buff_y'] = []
                self.curve[key]['buff_z'] = []
            #buff_x = self.buff_x
            #buff_y = self.buff_y
            #self.buff_x = []
            #self.buff_y = []
        finally:
            self.lock.release()
        return self.curve
    
    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            try:
                #data_x , data_y = self.next()
                self.next()
                for key in self.curve:
                    if self.curve[key]['enable']:
                        self.data_plot.update_values(self.curve[key]['topic_name'] , self.curve[key]['buff_x_temp'] , self.curve[key]['buff_y_temp'] , self.curve[key]['buff_z_temp'])                
                #if data_x or data_y:
                    #self.data_plot.update_values(self._topic_name , data_x , data_y)
                needs_redraw = True
            except RosPlotException as e:
                qWarning('PlotWidget : update_plot(): error in rosplot %s '%e)
        if needs_redraw:
            self.data_plot.redraw()

    @Slot(bool)
    def on_checkBox_auto_clicked(self, value):
        if value:
            self.enable_timer(enabled= True)
        else:
            self.enable_timer(enabled= False)
            
    def enable_timer(self , enabled = False):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
            
    def clean_up_subscribers(self):
        self._motor_subscriber.unregister()
        for key in self.curve:
            self.data_plot.remove_curve(self.curve[key]['topic_name'])
            
        
            
