import os
import rospy
import rospkg

import threading

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

#PATH = '/home/zhangzhi/catkin_ws/src/dragon_robot/dragon_visualization/dragon_button/src/dragon_button/'

class DragonButton(Plugin):

    def __init__(self, context):
        super(DragonButton, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DragonButton')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('dragon_button'), 'resource', 'button.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DragonButton')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        
        #threading
        self.lock = threading.Lock()        
        #widget
        self._widget.pushButton_start.clicked.connect(self.pushButton_start)
        self._widget.pushButton_start.setStyleSheet("background-color: rgb(128,255,0)")
        self._widget.pushButton_rviz.clicked.connect(self.pushButton_rviz)
        self._widget.pushButton_ok.setStyleSheet("background-color:rgb(128, 255, 0)")
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
    def pushButton_rviz(self):
        def system_rviz():
            try:
                os.system("rosrun rviz rviz")
            except:
                print "open rviz failed"
        try:
            self.lock.acquire()
            t_rviz = threading.Thread(target= system_rviz)
            t_rviz.setDaemon(True)
            t_rviz.start()
        except:
            print "threading rviz wrong"
        finally:
            self.lock.release()
    def pushButton_start(self):
        def system_launch():
            try:
                os.system("rosrun rqt_launch rqt_launch")
            except:
                print "open rqt_launch failed"
        if "Start" == self._widget.pushButton_start.text():
            try:
                self.lock.acquire()
                t_launch = threading.Thread(target= system_launch)
                t_launch.setDaemon(True)
                t_launch.start()
            except:
                print "threadig launch wrong"
            finally:
                self.lock.release()
            self._widget.pushButton_start.setText('Stop')
            self._widget.pushButton_start.setStyleSheet("background-color:rgb(255,0,0)")
        else:
            self._widget.pushButton_start.setText('Start')
            self._widget.pushButton_start.setStyleSheet("background-color:rgb(128,255,0)")
            try:
                file_name = rospkg.RosPack().get_path('dragon_button')+ '/src/dragon_button/rqt.txt'
                os.system('rm -f '+file_name)
                os.system('ps -ef |grep rqt_launch >>'+ file_name)
                for line in open(file_name).readlines():
                    if 'opt/ros/indigo' in line:
                        rqt_pid = line.split()[1]
                        os.system('kill -9 '+ rqt_pid)
            except:
                print "can't catch the rqt_launch data! "
                           