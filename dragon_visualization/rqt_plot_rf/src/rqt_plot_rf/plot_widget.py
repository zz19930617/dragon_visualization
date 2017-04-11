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

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

MOTOR_TOPIC_NAME = "/dragon/joint_angle_command"
ENCORDER_TOPIC_NAME = "/joint_states"
JOINT_DATA = ['hip_motor' , 'knee_motor' , 'yaw_motor' , 'hip_encorder' , 'knee_encorder'  , 'yaw_encorder']

class RosPlotException(Exception):
    pass

class PlotWidget(QWidget):
    _redraw_interval = 100
    def __init__(self):
        super(PlotWidget , self).__init__()
        self.setObjectName('PlotWidget')
        
        #ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_plot_rf') , 'resource' , 'plot.ui')
        loadUi(ui_file , self)
        
        #subscribe
        self._motor_topic_name = MOTOR_TOPIC_NAME
        self._encorder_topic_name = ENCORDER_TOPIC_NAME
        try:
            self._motor_subscriber = rospy.Subscriber(self._motor_topic_name , Float64MultiArray , self._motor_cb)
        except ValueError,e:
            rospy.logerr('rqt_plot: error connect topic (%s)'%e)
        try:
            self._encorder_subscriber = rospy.Subscriber(self._encorder_topic_name , JointState , self._encorder_cb)
        except ValueError,e:
            rospy.logerr('rqt_plot: error connect topic (%s)'%e)
            
        #timer
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        
        #variable
        self._joint_data = JOINT_DATA
        self.curve = {}
        for i in range(len(self._joint_data)):
            self.curve[self._joint_data[i]] = {'topic_name':self._joint_data[i] , 'buff_x':[] , 'buff_y':[] , 'buff_x_temp':[] , 'buff_y_temp' : [] ,'enable' : False}
        self.lock = threading.Lock()
        self.start_time = rospy.get_time()
        
        self.data_plot = None
        self.error = None

        self._if_click = {}
        checkBox = ['hip','knee']
        for key in checkBox:
            self._if_click[key] = False
        #print self._if_click
        
        
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
                self.data_plot.add_curve(self.curve[key]['topic_name'] , self.curve[key]['topic_name'] , self.curve[key]['buff_x_temp'] , self.curve[key]['buff_y_temp'])
        #self.enable_timer(enabled= True)
        self.data_plot.redraw()
        
    def _motor_cb(self , msg):
        try:
            self.lock.acquire()
            try:
                #msg.data[1] is data of left_front_hip
                #self.buff_y.append(msg.data[0])
                    self.curve['hip_motor']['buff_y'].append(msg.data[0])
                    self.curve['knee_motor']['buff_y'].append(msg.data[1])
                    if msg.__class__._has_header:
                        self.curve['hip_motor']['buff_x'].append(msg.header.stamp.to_sec() - self.start_time)
                        self.curve['knee_motor']['buff_x'].append(msg.header.stamp.to_sec() - self.start_time)
                        #self.buff_x.append(msg.header.stamp.to_sec() - self.start_time)
                    else:
                        self.curve['hip_motor']['buff_x'].append(rospy.get_time() - self.start_time)
                        self.curve['knee_motor']['buff_x'].append(rospy.get_time() - self.start_time)
                    #self.buff_x.append(rospy.get_time() - self.start_time)
            except AttributeError as e:
                self.error = RosPlotException("invalid topic data")
        
        finally:
            self.lock.release()
    
    def _encorder_cb(self , msg):
        try:
            self.lock.acquire()
            try:
                if msg.position is not []:
                    self.curve['hip_encorder']['buff_y'].append(msg.position[0])
                    self.curve['knee_encorder']['buff_y'].append(msg.position[1])
                #if msg.__class__._has_header:
                    #self.curve['hip_encorder']['buff_x'].append(msg.header.stamp.to_sec() - self.start_time)
                    #self.curve['knee_encorder']['buff_x'].append(msg.header.stamp.to_sec() - self.start_time)
                    #self.buff_x.append(msg.header.stamp.to_sec() - self.start_time)
                #else:
                    self.curve['hip_encorder']['buff_x'].append(rospy.get_time() - self.start_time)
                    self.curve['knee_encorder']['buff_x'].append(rospy.get_time() - self.start_time)
            except AttributeError as e:
                self.error = RosPlotException('invalid topic data in encorder')
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
                self.curve[key]['buff_x'] = []
                self.curve[key]['buff_y'] = []
        finally:
            self.lock.release()
    
    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            try:
                self.next()
                for key in self.curve:
                    if self.curve[key]['enable']:              
                        self.data_plot.update_values(self.curve[key]['topic_name'],self.curve[key]['buff_x_temp'],self.curve[key]['buff_y_temp'])
                needs_redraw = True
            except RosPlotException as e:
                qWarning('PlotWidget : update_plot(): error in rosplot %s '%e)
        if needs_redraw:
            self.data_plot.redraw()
    
    @Slot(bool)
    def on_checkBox_hip_clicked(self , value):
        if value:
            self._if_click['hip'] = True
            self.curve['hip_motor']['enable'] = value
            self.curve['hip_encorder']['enable'] = value
            self.enable_timer(enabled= True)
            self.next()
            self.data_plot.add_curve(self.curve['hip_motor']['topic_name'] , self.curve['hip_motor']['topic_name'] , self.curve['hip_motor']['buff_x_temp'] , self.curve['hip_motor']['buff_y_temp'])           
            self.data_plot.add_curve(self.curve['hip_encorder']['topic_name'] , self.curve['hip_encorder']['topic_name'] , self.curve['hip_encorder']['buff_x_temp'] , self.curve['hip_encorder']['buff_y_temp'])
        else:
            self._if_click['hip'] = False
            self.curve['hip_motor']['enable'] = value
            self.curve['hip_encorder']['enable'] = value
            self.data_plot.remove_curve(self.curve['hip_motor']['topic_name']) 
            self.data_plot.remove_curve(self.curve['hip_encorder']['topic_name'])
            self.update_plot()
            if True not in self._if_click.values():
                self.enable_timer(enabled= False)  
    
    @Slot(bool)
    def on_checkBox_knee_clicked(self , value):
        if value:
            self._if_click['knee'] = True
            self.curve['knee_motor']['enable'] = value
            self.curve['knee_encorder']['enable'] = value
            self.enable_timer(enabled= True)
            self.next()
            self.data_plot.add_curve(self.curve['knee_motor']['topic_name'] , self.curve['knee_motor']['topic_name'] , self.curve['knee_motor']['buff_x_temp'] , self.curve['knee_motor']['buff_y_temp'])
            self.data_plot.add_curve(self.curve['knee_encorder']['topic_name'] , self.curve['knee_encorder']['topic_name'] , self.curve['knee_encorder']['buff_x_temp'] , self.curve['knee_encorder']['buff_y_temp'])
        else:
            self._if_click['knee'] = False
            self.curve['knee_motor']['enable'] = value
            self.curve['knee_encorder']['enable'] = value
            self.data_plot.remove_curve(self.curve['knee_motor']['topic_name'])
            self.data_plot.remove_curve(self.curve['knee_encorder']['topic_name'])
            self.update_plot()
            if True not in self._if_click.values():
                self.enable_timer(enabled= False)  
            
    def enable_timer(self , enabled = True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
            
    def clean_up_subscribers(self):
        self._motor_subscriber.unregister()
        for key in self.curve:
            self.data_plot.remove_curve(self.curve[key]['topic_name'])
            
        
            