import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QBasicTimer
import time

import threading
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

MAX_VALUE = 3
TOPIC_NAME = "/debug"
SUB_NAME = "/joint_states"

LEG_NAME = ['L-F' , 'L-B' , 'R-F' , 'R-B']
DATA_TPYE = ['Position' , 'Velocity' , 'Effort']
JOINT_NAME = ['hip', 'knee', 'yaw']
PATH = '/home/robot/catkin_ws/src/dragon_visualization/dragon_data_control/src/dragon_data_control/data.txt'
class DragonDataControl(Plugin):

    def __init__(self, context):
        super(DragonDataControl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DragonDataControl')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('dragon_data_control'), 'resource', 'data_control.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DragonDataControl')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
            
        #init variable
        self.dragon_pointer = {}
        self.joint_name = JOINT_NAME
        for key in self.joint_name:
            self.dragon_pointer[key] = {'name': key , 'value': 0.0}
        self.factor = MAX_VALUE/100.0
        self.update_lineEdit()
        self._if_edit = True
        self._if_slider = True
        self._if_load = False

        self.hip_data = []
        self.knee_data = []
        self.lock = threading.Lock()
        self.sub_msg_pos = []
        self.sub_msg_vel = []

        
        #rospub
        self.topic_name = TOPIC_NAME
        self.sub_name = SUB_NAME
	try:
	    self._publisher_command = rospy.Publisher(self.topic_name, Float64MultiArray , queue_size=10)
	except ROSException, e:
	    rospy.logerr('Dragon_data_control: Error creating publisher for topic %s (%s)'%(self._pub_topic, e))
        try:
            self.sub_command = rospy.Subscriber(self.sub_name, JointState, self.sub_cb)
        except ValueError, e:
            rospy.logerr('Dragon_data_control: Error connecting topic (%s)'%e)
                    
        #widget
        self.leg_name = LEG_NAME
        self.data_tpye = DATA_TPYE
        for key in self.leg_name:
            self._widget.comboBox_leg.addItem(key)
        for key in self.data_tpye:
            self._widget.comboBox_type.addItem(key)
            
        self._widget.horizontalSlider_hip.valueChanged.connect(self.slider_changed)
        self._widget.horizontalSlider_knee.valueChanged.connect(self.slider_changed)
        self._widget.horizontalSlider_yaw.valueChanged.connect(self.slider_changed)
        self._widget.lineEdit_hip.textChanged.connect(self.lineEdit_changed)
        self._widget.lineEdit_knee.textChanged.connect(self.lineEdit_changed)
        self._widget.lineEdit_yaw.textChanged.connect(self.lineEdit_changed)
        
        self._widget.pushButton_reset.clicked.connect(self.pushButton_reset)
        self._widget.pushButton_go.clicked.connect(self.pushButton_go)
        self._widget.pushButton_load.clicked.connect(self.pushButton_load)
        self._widget.pushButton_stop.clicked.connect(self.pushButton_stop)
        self._widget.pushButton_get.clicked.connect(self.pushButton_get)
        
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        #timer
        
    def slider_changed(self):
        if self._if_slider:
            self.dragon_pointer['hip']['value'] = self._widget.horizontalSlider_hip.value() * self.factor
            self.dragon_pointer['knee']['value'] = self._widget.horizontalSlider_knee.value() * self.factor
            self.dragon_pointer['yaw']['value'] = self._widget.horizontalSlider_yaw.value() * self.factor
            self.update_lineEdit()
        self._if_slider = True
    
    def lineEdit_changed(self):
        if self._if_edit:
            self.dragon_pointer['hip']['value'] = float(self._widget.lineEdit_hip.text())
            self.dragon_pointer['knee']['value'] = float(self._widget.lineEdit_knee.text())
            self.dragon_pointer['yaw']['value'] = float(self._widget.lineEdit_yaw.text())
            self.update_slider()
        self._if_edit = True
    
    def update_lineEdit(self):
        self._widget.lineEdit_hip.setText(str(round(self.dragon_pointer['hip']['value'], 3)))
        self._widget.lineEdit_knee.setText(str(round(self.dragon_pointer['knee']['value'], 3)))
        self._widget.lineEdit_yaw.setText(str(round(self.dragon_pointer['yaw']['value'], 3)))
        self._if_edit = False
        
    def update_slider(self):
        self._widget.horizontalSlider_hip.setValue(self.dragon_pointer['hip']['value']/self.factor)
        self._widget.horizontalSlider_knee.setValue(self.dragon_pointer['knee']['value']/self.factor)
        self._widget.horizontalSlider_yaw.setValue(self.dragon_pointer['yaw']['value']/self.factor)
        self._if_slider = False
        
    def pushButton_reset(self):
        self._if_edit = False
        self._if_slider = False
        self._widget.lineEdit_hip.setText(str(0.0))
        self._widget.lineEdit_knee.setText(str(0.0))
        self._widget.lineEdit_yaw.setText(str(0.0))
        
    def pushButton_go(self):
        current_leg = self._widget.comboBox_leg.currentText()
	current_type = self._widget.comboBox_type.currentText()
	joint_data = Float64MultiArray()
	joint_data.data = [0, 0, 0, 0, 0 , 0, 0, 0]	
	if "Position" == current_type:
	    if "L-F" == current_leg:
		joint_data.data[0] = self.dragon_pointer['hip']['value']
		joint_data.data[1] = self.dragon_pointer['knee']['value']
	    elif "L-B" == current_leg:
		joint_data.data[2] = self.dragon_pointer['hip']['value']
		joint_data.data[3] = self.dragon_pointer['knee']['value']
	    elif "R-F" == current_leg:
		joint_data.data[4] = self.dragon_pointer['hip']['value']
		joint_data.data[5] = self.dragon_pointer['knee']['value']	
	    elif "R-B" == current_leg:
		joint_data.data[6] = self.dragon_pointer['hip']['value']
		joint_data.data[7] = self.dragon_pointer['knee']['value']
		
	self._publisher_command.publish(joint_data)

    def sub_cb(self,msg):
        self.sub_msg_pos = msg.position
        self.sub_msg_vel = msg.velocity

    def pushButton_get(self):
        current_leg = self._widget.comboBox_leg.currentText()
        current_type = self._widget.comboBox_type.currentText()
        if "Position" == current_type:
            if "L-F" == current_leg:
                self.dragon_pointer['hip']['value'] = self.sub_msg_pos[0]
                self.dragon_pointer['knee']['value'] = self.sub_msg_pos[1]
            elif 'L-B' == current_leg:
                self.dragon_pointer['hip']['value'] = self.sub_msg_pos[2]
                self.dragon_pointer['knee']['value'] = self.sub_msg_pos[3]
            elif 'R-F' == current_leg:
                self.dragon_pointer['hip']['value'] = self.sub_msg_pos[4]
                self.dragon_pointer['knee']['value'] = self.sub_msg_pos[5]
            elif 'R-B' == current_leg:
                self.dragon_pointer['hip']['value'] = self.sub_msg_pos[6]
                self.dragon_pointer['knee']['value'] = self.sub_msg_pos[7]
        elif 'Velocity' == current_type:
            if "L-F" == current_leg:
                self.dragon_pointer['hip']['value'] = self.sub_msg_vel[0]
                self.dragon_pointer['knee']['value'] = self.sub_msg_vel[1]
        self.update_lineEdit()

    def data_pub(self):
        while(self._if_load):
            msg = Float64MultiArray()
            msg.data = [0,0,0, 0, 0, 0, 0, 0]
            for i in range(len(self.hip_data)):
                if self._if_load:
                    msg.data[4] = self.hip_data[i]
                    #msg.data[5] = self.knee_data[i]
                    self._publisher_command.publish(msg)
                    time.sleep(0.05)

    def pushButton_load(self):
        try:
            with open(PATH,'r') as f:
                data = f.readlines()
                hip_data_str = data[0].split(',')
                self.hip_data = [float(i) for i in hip_data_str]
                knee_data_str = data[1].split(',')
                self.knee_data = [float(i) for i in knee_data_str]
                self._if_load = True
        except:
            print 'load txt wrong! '
        try:
            self.lock.acquire()
            self.data_thread = threading.Thread(target=self.data_pub)
            self.data_thread.setDaemon(True)
            self.data_thread.start()
        except:
            print 'thread is wrong!'
        finally:
            self.lock.release()

    def pushButton_stop(self):
        self._if_load = False

