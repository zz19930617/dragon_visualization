from __future__ import division
import os
import rospkg
import threading

import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot, QBasicTimer, SIGNAL
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget, QPixmap, QMessageBox, QStandardItemModel,QStandardItem
from rqt_gui_py.plugin import Plugin

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import time
import math

#topic_name
CONTROLLER_TOPIC = "/dragon/joint_angle_command"
ENCORDER_TOPIC = "/joint_states"
ELE_CUR_TOPIC = "/cur_states"

#joint_index
JOINT_INDEX = ['lbh', 'lbk','lby', 'lfh', 'lfk', 'lfy', 'rbh', 'rbk','rby', 'rfh', 'rfk','rfy']
CUR_INDEX = ['lf_cur', 'lb_cur', 'rf_cur', 'rb_cur']

class DataRead(Plugin):
    
    def __init__(self,context):
        super(DataRead,self).__init__(context)
        self.setObjectName('DataRead')
                    
        #ui
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('dragon_data_read'),'resource','DragonDataRead.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('DataRead')
               
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.lock = threading.Lock()

        '''
        structure of dragon_joint
        dragon_joint_pointer = {
            "lfh":{
                dragon_joint_name:"left_front_hip"
                sensor:"motor"{
                    position:0
                    velocity:0
                    effort:0
                }
            }
        }
        '''
        self.dragon_joint_index = JOINT_INDEX
        self.cur_index = CUR_INDEX
        self.dragon_joint_pointer = {}
        for i in range(len(self.dragon_joint_index)):
            self.dragon_joint_pointer[self.dragon_joint_index[i]] = {"joint_name":self.dragon_joint_index[i] , "motor": {"position" : 0 , "velocity" : 0 , "effort" : 0} , "encorder" : {"position" : 0 , "velocity" : 0 , "effort" : 0}}
            lcdNumber_name = 'lcdNumber_' + self.dragon_joint_index[i] + 'm' + '_pos'
            getattr(self._widget,lcdNumber_name).setStyleSheet("background-color: rgb(176,196,222)")
            lcdNumber_name = 'lcdNumber_' + self.dragon_joint_index[i] + 'e' + '_pos'
            getattr(self._widget,lcdNumber_name).setStyleSheet("background-color: rgb(255,182,193)")
        for i in range(len(self.cur_index)):
            lcdNumber_name = 'lcdNumber_' + self.cur_index[i]
            getattr(self._widget,lcdNumber_name).setStyleSheet("background-color : rgb(192,192,192)")            
        #init rostopic to connect
        self._motor_topic = CONTROLLER_TOPIC
        self._encorder_topic = ENCORDER_TOPIC
        self._ele_cur_topic = ELE_CUR_TOPIC
        
        #subscribe
        try:
            self._motor_subscriber = rospy.Subscriber(self._motor_topic, Float64MultiArray , self._receive_motor_data)
        except ValueError, e:
            rospy.logerr('dragon_data_read: Error connecting topic (%s)'%e)
        try:
            self._encorder_subscriber = rospy.Subscriber(self._encorder_topic, JointState , self._receive_encorder_data)
        except ValueError, e:
            rospy.logerr('dragon_data_read: Error connecting topic (%s)'%e)
        try:
            self.ele_cur_subscriber = rospy.Subscriber(self._ele_cur_topic, Float64MultiArray , self._receive_cur_data)
        except ValueError, e:
            rospy.logerr('dragon_data_read: Error connecting topic (%s)'%e)

        #timer
        #self._init_timers()

    def _receive_motor_data(self, msg):
        #self.lock.acquire()
        for i in range(len(msg.data)):
            self.dragon_joint_pointer[self.dragon_joint_index[i]]['motor']['position'] = msg.data[i]
            lcdNumber_name = 'lcdNumber_' + self.dragon_joint_index[i] + 'm' + '_pos'
            getattr(self._widget,lcdNumber_name).display(self.dragon_joint_pointer[self.dragon_joint_index[i]]['motor']['position'])
        #self.lock.release()
            
    def _receive_encorder_data(self, msg):
        #self.lock.acquire()
        for i in range(len(msg.position)):
            self.dragon_joint_pointer[self.dragon_joint_index[i]]['encorder']['position'] = msg.position[i]
            lcdNumber_name = 'lcdNumber_' + self.dragon_joint_index[i] + 'e' + '_pos'
            getattr(self._widget,lcdNumber_name).display(self.dragon_joint_pointer[self.dragon_joint_index[i]]['encorder']['position'])

    def _receive_cur_data(self, msg):
        for i in range(len(self.cur_index)):
            lcdNumber_name = 'lcdNumber_' + self.cur_index[i]
            getattr(self._widget, lcdNumber_name).display(msg.data[i])
    
    def shutdown_plugin(self):
        #self._timer.stop()
        self._motor_subscriber.unregister()
        self._encorder_subscriber.unregister()
        

