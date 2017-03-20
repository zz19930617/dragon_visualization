import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from std_msgs.msg import Float64MultiArray

MAX_VALUE = 3
TOPIC_NAME = "/debug"

LEG_NAME = ['L-F' , 'L-B' , 'R-F' , 'R-B']
DATA_TPYE = ['Position' , 'Velocity' , 'Effort']
JOINT_NAME = ['hip', 'knee', 'yaw']
class DragonDataControl(Plugin):

    def __init__(self, context):
        super(DragonDataControl, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DragonDataControl')

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

        
        #rospub
        self.topic_name = TOPIC_NAME
	try:
	    self._publisher_command = rospy.Publisher(self.topic_name, Float64MultiArray , queue_size=10)
	except ROSException, e:
	    rospy.logerr('Dragon_data_control: Error creating publisher for topic %s (%s)'%(self._pub_topic, e))
                    
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
        
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
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

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog