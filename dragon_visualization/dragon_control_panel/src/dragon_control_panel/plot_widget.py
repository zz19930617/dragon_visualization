import os
import rospkg
import threading
import roslib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QAction, QMenu, QWidget, QFileDialog

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

import rospy

from rqt_py_common.topic_completer import TopicCompleter
from rqt_py_common import topic_helpers

MOTOR_TOPIC_NAME = "/dragon/joint_angle_command"
ENCORDER_TOPIC_NAME = "/joint_states"
JOINT_DATA = ['hip_motor' , 'knee_motor' , 'yaw_motor' , 'hip_encorder' , 'knee_encorder'  , 'yaw_encorder']

LEG_NAME =['L-F' , 'L-B' , 'R-F' ,'R-B']
JOINT_NAME = ['hip' , 'knee' ,'yaw']
class RosPlotException(Exception):
    pass

class PlotWidget(QWidget):
    _redraw_interval = 40
    def __init__(self):
        super(PlotWidget , self).__init__()
        self.setObjectName('PlotWidget')
        
        #ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('dragon_control_panel') , 'resource' , 'plot.ui')
        loadUi(ui_file , self)
            
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
        self.txt_list = []
        self._if_pause = False
        self._if_load = False
        
        #widget
        self.leg_name = LEG_NAME
        self.joint_name = JOINT_NAME
        for key in self.leg_name:
            self.comboBox_leg.addItem(key)
        for key in self.joint_name:
            self.comboBox_joint.addItem(key)
        self.pushButton_open.clicked.connect(self.pB_open)
        self.pushButton_load.clicked.connect(self.pB_load)
        #self.label_start.setStyleSheet("background-color: rgb(128,255,0)")
        #print dir(self.pushButton_start)
        self.pushButton_start.setStyleSheet("background-color: rgb(128,255,0)")
        self.label_on.setStyleSheet("background-color:rgb(255,0,0)")
        self.pushButton_start.clicked.connect(self.pB_start)
        self.pushButton_pause.clicked.connect(self.pB_pause)
             
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
        self.enable_timer(enabled= True)
        self.data_plot.redraw()
     
    def pB_open(self):
        filename,filetype = QFileDialog.getOpenFileName(self, "pick document", "/home",)
        #print filetype
        self.lineEdit_filename.setText(str(filename))
    def pB_load(self):
        filename = self.lineEdit_filename.text()
        final_file = filename.split('/')[-1]
        #print filename
        if ".txt" in filename:
            self._if_load = True
            self.data_plot.add_curve(self.curve['hip_motor']['topic_name'], final_file ,self.curve['hip_motor']['buff_x_temp'], self.curve['hip_motor']['buff_y_temp'])    
            for line in open(filename):
                try:
                    self.txt_list.append(float(line))                   
                except:
                    print "the txt_data is not a number"            
        else:
            print 'this is wrong document'
                 
    def pB_start(self):
        if 'Start' == self.pushButton_start.text():
            if self._if_load:
                self.curve['hip_motor']['enable'] = True           
            self.pushButton_start.setText('Stop')
            self.pushButton_start.setStyleSheet("background-color: rgb(255,0,0)")
            self.label_on.setText('ON')
            self.label_on.setStyleSheet("background-color: rgb(128,255,0)")
        elif 'Stop' == self.pushButton_start.text():
            self._if_load = False
            self.curve['hip_motor']['enable'] = False
            self.pushButton_start.setText('Start')
            self.pushButton_start.setStyleSheet("background-color: rgb(128,255,0)")
            self.label_on.setText('OFF')
            self.label_on.setStyleSheet("background-color: rgb(255,0,0)")            
            try:
                self.data_plot.remove_curve('hip_motor')
            except:
                print "curve dosen't exist"
                
    def pB_pause(self):
        self._if_pause = not self._if_pause
        if "PAUSE" == self.pushButton_pause.text():
            self.pushButton_pause.setText('RE-PLOT')
        else:
            self.pushButton_pause.setText('PAUSE')
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
                if not self._if_pause:
                    for each_data in self.txt_list:
                        self.curve['hip_motor']['buff_x_temp'].append(rospy.get_time() - self.start_time)
                        self.curve['hip_motor']['buff_y_temp'].append(each_data)
                    for key in self.curve:
                        if self.curve[key]['enable']:
                            self.data_plot.update_values(self.curve[key]['topic_name'] , self.curve[key]['buff_x_temp'] , self.curve[key]['buff_y_temp'])
                    needs_redraw = True
                    self.curve['hip_motor']['buff_x_temp'] = []
                    self.curve['hip_motor']['buff_y_temp'] = []
            except RosPlotException as e:
                qWarning('PlotWidget : update_plot(): error in rosplot %s '%e)
        if needs_redraw:
            self.data_plot.redraw()
                
    def enable_timer(self , enabled = True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
            
    def clean_up_subscribers(self):
        for key in self.curve:
            self.data_plot.remove_curve(self.curve[key]['topic_name'])
            
        
            