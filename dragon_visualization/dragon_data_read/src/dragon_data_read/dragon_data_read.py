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

#joint_index
JOINT_INDEX = ['lfh', 'lfk','lbh', 'lbk','rfh', 'rfk','rbh', 'rbk',]
#joint_name
JOINT_NAME = ['left_front_hip', 'left_front_knee','left_back_hip', 'left_back_knee','right_front_hip', 'right_front_knee', 'right_back_hip', 'right_back_knee']

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
        self.dragon_joint_name = JOINT_NAME
        self.dragon_joint_pointer = {}
        for i in range(len(self.dragon_joint_index)):
            self.dragon_joint_pointer[self.dragon_joint_index[i]] = {"joint_name":self.dragon_joint_name[i] , "motor": {"position" : 0 , "velocity" : 0 , "effort" : 0} , "encorder" : {"position" : 0 , "velocity" : 0 , "effort" : 0}}
            lcdNumber_name = 'lcdNumber_' + self.dragon_joint_index[i] + 'm' + '_pos'
            getattr(self._widget,lcdNumber_name).setStyleSheet("background-color: rgb(176,196,222)")
            lcdNumber_name = 'lcdNumber_' + self.dragon_joint_index[i] + 'e' + '_pos'
            getattr(self._widget,lcdNumber_name).setStyleSheet("background-color: rgb(255,182,193)")
            
        #init rostopic to connect
        self._motor_topic = CONTROLLER_TOPIC
        self._encorder_topic = ENCORDER_TOPIC
        
        #subscribe
        try:
            self._motor_subscriber = rospy.Subscriber(self._motor_topic, Float64MultiArray , self._receive_motor_data)
        except ValueError, e:
            rospy.logerr('BHandGUI: Error connecting topic (%s)'%e)
        try:
            self._encorder_subscriber = rospy.Subscriber(self._encorder_topic, JointState , self._receive_encorder_data)
        except ValueError, e:
            rospy.logerr('BHandGUI: Error connecting topic (%s)'%e)

        #timer
        #self._init_timers()

    def _receive_motor_data(self, msg):
        temp = msg
        #self.lock.acquire()
        #for i in range(len(msg.data)):
            #self.dragon_joint_pointer[self.dragon_joint_index[i]]['motor']['position'] = msg.data[i]
            #lcdNumber_name = 'lcdNumber_' + self.dragon_joint_index[i] + 'm' + '_pos'
            #getattr(self._widget,lcdNumber_name).display(self.dragon_joint_pointer[self.dragon_joint_index[i]]['motor']['position'])
        #self.lock.release()
            
    def _receive_encorder_data(self, msg):
        #self.lock.acquire()
        #for i in range(len(msg.position)):
            #self.dragon_joint_pointer[self.dragon_joint_index[i]]['encorder']['position'] = msg.position[i]
            #lcdNumber_name = 'lcdNumber_' + self.dragon_joint_index[i] + 'e' + '_pos'
            #getattr(self._widget,lcdNumber_name).display(self.dragon_joint_pointer[self.dragon_joint_index[i]]['encorder']['position'])
        #self.lock.acquire()
    #def _init_timers(self):
        #self._timer = QBasicTimer()
        #self._timer.start(1,self)

        self.dragon_joint_pointer['rfh']['encorder']['position'] = msg.position[9]
        self.dragon_joint_pointer['rfk']['encorder']['position'] = msg.position[10]
        lcdNumber_name = 'lcdNumber_' + 'rfh' + 'e' + '_pos'
        getattr(self._widget,lcdNumber_name).display(self.dragon_joint_pointer['rfh']['encorder']['position'])
        lcdNumber_name = 'lcdNumber_' + 'rfk' + 'e' + '_pos'
        getattr(self._widget,lcdNumber_name).display(self.dragon_joint_pointer['rfk']['encorder']['position'])
    def shutdown_plugin(self):
        #self._timer.stop()
        self._motor_subscriber.unregister()
        self._encorder_subscriber.unregister()
        
    #def timerEvent(self, e):
        
        #for key in self.dragon_joint_pointer:
            #read motor_position_data
            #lcdNumber_name = 'lcdNumber_' + key + 'm' + '_pos'
            #getattr(self._widget,lcdNumber_name).display(self.dragon_joint_pointer[key]['motor']['position'])
            #getattr(self._widget,lcdNumber_name).setStyleSheet("background-color: rgb(176,196,222)")
            #read encorder_position_data
            #lcdNumber_name = 'lcdNumber_' + key + 'e' + '_pos'
            #getattr(self._widget,lcdNumber_name).display(self.dragon_joint_pointer[key]['encorder']['position'])
            #getattr(self._widget,lcdNumber_name).setStyleSheet("background-color: rgb(255,182,193)")
