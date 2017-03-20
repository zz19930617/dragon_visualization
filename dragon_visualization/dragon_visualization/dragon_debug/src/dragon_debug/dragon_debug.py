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

MAX_VALUE = 3

TOPCI_NAME = "/debug"
#joint_index
JOINT_INDEX = ['lfh', 'lfk', 'lbh', 'lbk', 'rfh', 'rfk', 'rbh', 'rbk']
#joint_name
JOINT_NAME = ['left_front_hip', 'left_front_knee', 'left_back_hip', 'left_back_knee', 'right_front_hip', 'right_front_knee', 'right_back_hip', 'right_back_knee']

class DragonDebug(Plugin):

    def __init__(self, context):
        super(DragonDebug, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DragonDebug')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('dragon_debug'), 'resource', 'DragonDebug.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DragonDebug')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
	
	#structure of joint_data
	#dragon_joint_data{
	#"name":"jont_name"
	#"value":data
	#}
	self.dragon_joint_point = {}
	self.dragon_joint_index = JOINT_INDEX
	self.dragon_joint_name = JOINT_NAME
	for i in range(len(self.dragon_joint_index)):
	    self.dragon_joint_point[self.dragon_joint_index[i]] = {"name" : self.dragon_joint_name[i] , "value" : 0.0}
	
        #init variable
        self.factor = MAX_VALUE/100
	self._joint_data = Float64MultiArray()
	self._updateGUI()
	
	#flag
	self._if_edit = True
	self._if_slider = True
	#rostopic to pub
	self._pub_topic = TOPCI_NAME
	try:
	    self._publisher_command = rospy.Publisher(self._pub_topic, Float64MultiArray , queue_size=10)
	except ROSException, e:
	    rospy.logerr('DragonDebugGUI: Error creating publisher for topic %s (%s)'%(self._pub_topic, e))
	
        #HANDLES
	#self._widget.lineEdit_lfh.textChanged.connect(self.lineEdit_changed_lfh)
	#self._widget.lineEdit_lfk.textChanged.connect(self.lineEdit_changed_lfk)
	#self._widget.lineEdit_lbh.textChanged.connect(self.lineEdit_changed_lbh)
	#self._widget.lineEdit_lbk.textChanged.connect(self.lineEdit_changed_lbk)
	#self._widget.lineEdit_rfh.textChanged.connect(self.lineEdit_changed_rfh)
	#self._widget.lineEdit_rfk.textChanged.connect(self.lineEdit_changed_rfk)
	#self._widget.lineEdit_rbh.textChanged.connect(self.lineEdit_changed_rbh)
	#self._widget.lineEdit_rbk.textChanged.connect(self.lineEdit_changed_rbk)
        #self._widget.horizontalSlider_lfh.valueChanged.connect(self.slider_changed_lfh)
	#self._widget.horizontalSlider_lfk.valueChanged.connect(self.slider_changed_lfk)
	#self._widget.horizontalSlider_lbh.valueChanged.connect(self.slider_changed_lbh)
	#self._widget.horizontalSlider_lbk.valueChanged.connect(self.slider_changed_lbk)
	#self._widget.horizontalSlider_rfh.valueChanged.connect(self.slider_changed_rfh)
	#self._widget.horizontalSlider_rfk.valueChanged.connect(self.slider_changed_rfk)
	#self._widget.horizontalSlider_rbh.valueChanged.connect(self.slider_changed_rbh)
	#self._widget.horizontalSlider_rbk.valueChanged.connect(self.slider_changed_rbk)
	self._widget.pushButton_ok.clicked.connect(self.ok_button_pressed)
	
        self._widget.horizontalSlider_lfh.valueChanged.connect(self.slider_changed)
	self._widget.horizontalSlider_lfk.valueChanged.connect(self.slider_changed)
	self._widget.horizontalSlider_lbh.valueChanged.connect(self.slider_changed)
	self._widget.horizontalSlider_lbk.valueChanged.connect(self.slider_changed)
	self._widget.horizontalSlider_rfh.valueChanged.connect(self.slider_changed)
	self._widget.horizontalSlider_rfk.valueChanged.connect(self.slider_changed)
	self._widget.horizontalSlider_rbh.valueChanged.connect(self.slider_changed)
	self._widget.horizontalSlider_rbk.valueChanged.connect(self.slider_changed)
	
	self._widget.lineEdit_lfh.textChanged.connect(self.lineEdit_changed)
	self._widget.lineEdit_lfk.textChanged.connect(self.lineEdit_changed)
	self._widget.lineEdit_lbh.textChanged.connect(self.lineEdit_changed)
	self._widget.lineEdit_lbk.textChanged.connect(self.lineEdit_changed)
	self._widget.lineEdit_rfh.textChanged.connect(self.lineEdit_changed)
	self._widget.lineEdit_rfk.textChanged.connect(self.lineEdit_changed)
	self._widget.lineEdit_rbh.textChanged.connect(self.lineEdit_changed)
	self._widget.lineEdit_rbk.textChanged.connect(self.lineEdit_changed)	
	
    def lineEdit_changed(self):
	if self._if_edit:
	    self.dragon_joint_point['lfh']['value'] = float(self._widget.lineEdit_lfh.text())
	    self.dragon_joint_point['lfk']['value'] = float(self._widget.lineEdit_lfk.text())
	    self.dragon_joint_point['lbh']['value'] = float(self._widget.lineEdit_lbh.text())
	    self.dragon_joint_point['lbk']['value'] = float(self._widget.lineEdit_lbk.text())
	    self.dragon_joint_point['rfh']['value'] = float(self._widget.lineEdit_rfh.text())
	    self.dragon_joint_point['rfk']['value'] = float(self._widget.lineEdit_rfk.text())
	    self.dragon_joint_point['rbh']['value'] = float(self._widget.lineEdit_rbh.text())
	    self.dragon_joint_point['rbk']['value'] = float(self._widget.lineEdit_rbk.text())
	    self._updateGUI_slider()
	self._if_edit = True
    #def lineEdit_changed_lfh(self):
	#if self._if_edit:
	    #self.dragon_joint_point['lfh']['value'] = float(self._widget.lineEdit_lfh.text())
	    #self._widget.horizontalSlider_lfh.setValue(self.dragon_joint_point['lfh']['value']/self.factor)
	#self._if_edit = True
	#self._if_slider = False
    #def lineEdit_changed_lfk(self):
	#self.dragon_joint_point['lfk']['value'] = float(self._widget.lineEdit_lfk.text())
    #def lineEdit_changed_lbh(self):
    	#self.dragon_joint_point['lbh']['value'] = float(self._widget.lineEdit_lbh.text())
    #def lineEdit_changed_lbk(self):
    	#self.dragon_joint_point['lbk']['value'] = float(self._widget.lineEdit_lbk.text())
    #def lineEdit_changed_rfh(self):
    	#self.dragon_joint_point['rfh']['value'] = float(self._widget.lineEdit_rfh.text())
    #def lineEdit_changed_rfk(self):
    	#self.dragon_joint_point['rfk']['value'] = float(self._widget.lineEdit_rfk.text())
    #def lineEdit_changed_rbh(self):
    	#self.dragon_joint_point['rbh']['value'] = float(self._widget.lineEdit_rbh.text())
    #def lineEdit_changed_rbk(self):
    	#self.dragon_joint_point['rbk']['value'] = float(self._widget.lineEdit_rbk.text())
    def slider_changed(self):
	if self._if_slider:
	    self.dragon_joint_point['lfh']['value'] = self._widget.horizontalSlider_lfh.value() * self.factor
	    self.dragon_joint_point['lfk']['value'] = self._widget.horizontalSlider_lfk.value() * self.factor
	    self.dragon_joint_point['lbh']['value'] = self._widget.horizontalSlider_lbh.value() * self.factor
	    self.dragon_joint_point['lbk']['value'] = self._widget.horizontalSlider_lbk.value() * self.factor
	    self.dragon_joint_point['rfh']['value'] = self._widget.horizontalSlider_rfh.value() * self.factor
	    self.dragon_joint_point['rfk']['value'] = self._widget.horizontalSlider_rfk.value() * self.factor
	    self.dragon_joint_point['rbh']['value'] = self._widget.horizontalSlider_rbh.value() * self.factor
	    self.dragon_joint_point['rbk']['value'] = self._widget.horizontalSlider_rbk.value() * self.factor
	    self._updateGUI()
	self._if_slider = True
    #def slider_changed_lfh(self):
	#if self._if_slider:
	    #self.dragon_joint_point['lfh']['value'] = self._widget.horizontalSlider_lfh.value() * self.factor
	    #self._updateGUI()
	#self._if_slider = True
    #def slider_changed_lfk(self):
	#self.dragon_joint_point['lfk']['value'] = self._widget.horizontalSlider_lfk.value() * self.factor
	#self._updateGUI()
    #def slider_changed_lbh(self):
	#self.dragon_joint_point['lbh']['value'] = self._widget.horizontalSlider_lbh.value() * self.factor
	#self._updateGUI()
    #def slider_changed_lbk(self):
	#self.dragon_joint_point['lbk']['value'] = self._widget.horizontalSlider_lbk.value() * self.factor
	#self._updateGUI()
    #def slider_changed_rfh(self):
	#self.dragon_joint_point['rfh']['value'] = self._widget.horizontalSlider_rfh.value() * self.factor
	#self._updateGUI()
    #def slider_changed_rfk(self):
	#self.dragon_joint_point['rfk']['value'] = self._widget.horizontalSlider_rfk.value() * self.factor
	#self._updateGUI()
    #def slider_changed_rbh(self):
	#self.dragon_joint_point['rbh']['value'] = self._widget.horizontalSlider_rbh.value() * self.factor
	#self._updateGUI()
    #def slider_changed_rbk(self):
	#self.dragon_joint_point['rbk']['value'] = self._widget.horizontalSlider_rbk.value() * self.factor
	#self._updateGUI()
    def ok_button_pressed(self):
	self._updateData()
	self._joint_data = Float64MultiArray()
	for i in self.dragon_joint_index:
	    self._joint_data.data.append(round(self.dragon_joint_point[i]['value'],3))
	#self._joint_data.data = [round(self.dragon_joint_point['lfh']['value'],3)]
	self._publisher_command.publish(self._joint_data)

    def shutdown_plugin(self):
	self._publisher_command.unregister()
    
    def _updateData(self):
	self.dragon_joint_point['lfh']['value'] = float(self._widget.lineEdit_lfh.text())
	self.dragon_joint_point['lfk']['value'] = float(self._widget.lineEdit_lfk.text())
	self.dragon_joint_point['lbh']['value'] = float(self._widget.lineEdit_lbh.text())
	self.dragon_joint_point['lbk']['value'] = float(self._widget.lineEdit_lbk.text())
	self.dragon_joint_point['rfh']['value'] = float(self._widget.lineEdit_rfh.text())
	self.dragon_joint_point['rfk']['value'] = float(self._widget.lineEdit_rfk.text())
	self.dragon_joint_point['rbh']['value'] = float(self._widget.lineEdit_rbh.text())
	self.dragon_joint_point['rbk']['value'] = float(self._widget.lineEdit_rbk.text())
    
    def _updateGUI(self):
	#update dragon_joint_point
	self._widget.lineEdit_lfh.setText(str(round(self.dragon_joint_point['lfh']['value'],3)))
	self._widget.lineEdit_lfk.setText(str(round(self.dragon_joint_point['lfk']['value'],3)))
	self._widget.lineEdit_lbh.setText(str(round(self.dragon_joint_point['lbh']['value'],3)))
	self._widget.lineEdit_lbk.setText(str(round(self.dragon_joint_point['lbk']['value'],3)))
	self._widget.lineEdit_rfh.setText(str(round(self.dragon_joint_point['rfh']['value'],3)))
	self._widget.lineEdit_rfk.setText(str(round(self.dragon_joint_point['rfk']['value'],3)))
	self._widget.lineEdit_rbh.setText(str(round(self.dragon_joint_point['rbh']['value'],3)))
	self._widget.lineEdit_rbk.setText(str(round(self.dragon_joint_point['rbk']['value'],3)))
	self._if_edit = False
    def _updateGUI_slider(self):
	self._widget.horizontalSlider_lfh.setValue(self.dragon_joint_point['lfh']['value']/self.factor)
	self._widget.horizontalSlider_lfk.setValue(self.dragon_joint_point['lfk']['value']/self.factor)
	self._widget.horizontalSlider_lbh.setValue(self.dragon_joint_point['lbh']['value']/self.factor)
	self._widget.horizontalSlider_lbk.setValue(self.dragon_joint_point['lbk']['value']/self.factor)
	self._widget.horizontalSlider_rfh.setValue(self.dragon_joint_point['rfh']['value']/self.factor)
	self._widget.horizontalSlider_rfk.setValue(self.dragon_joint_point['rfk']['value']/self.factor)
	self._widget.horizontalSlider_rbh.setValue(self.dragon_joint_point['rbh']['value']/self.factor)
	self._widget.horizontalSlider_rbk.setValue(self.dragon_joint_point['rbk']['value']/self.factor)
	self._if_slider = False
    
    #def timerEvent(self, e):
	#self._widget.lineEdit_lfh.setText(str(round(self.dragon_joint_point['lfh']['value'],3)))
	#self._widget.lineEdit_lfk.setText(str(round(self.dragon_joint_point['lfk']['value'],3)))
	#self._widget.lineEdit_lbh.setText(str(round(self.dragon_joint_point['lbh']['value'],3)))
	#self._widget.lineEdit_lbk.setText(str(round(self.dragon_joint_point['lbk']['value'],3)))
	#self._widget.lineEdit_rfh.setText(str(round(self.dragon_joint_point['rfh']['value'],3)))
	#self._widget.lineEdit_rfk.setText(str(round(self.dragon_joint_point['rfk']['value'],3)))
	#self._widget.lineEdit_rbh.setText(str(round(self.dragon_joint_point['rbh']['value'],3)))
	#self._widget.lineEdit_rbk.setText(str(round(self.dragon_joint_point['rbk']['value'],3)))
