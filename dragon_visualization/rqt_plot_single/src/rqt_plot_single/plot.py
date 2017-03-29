import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from .plot_widget import PlotWidget
from .rqt_plot_lf.src.rqt_plot_lf.plot import Plot as LFPlot
from .rqt_plot_lb.src.rqt_plot_lb.plot import Plot as LBPlot
from .rqt_plot_rf.src.rqt_plot_rf.plot import Plot as RFPlot
from .rqt_plot_rb.src.rqt_plot_rb.plot import Plot as RBPlot
class Plot(Plugin):

    def __init__(self, context):
        super(Plot, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SinglePlot')

        self._widget = PlotWidget()
        self.lf_plot = LFPlot()
        self.lb_plot = LBPlot()
        self.rf_plot = RFPlot()
        self.rb_plot = RBPlot()
        self._widget.switch_widget_lf(self.lf_plot)
        self._widget.switch_widget_lb(self.lb_plot)
        self._widget.switch_widget_rf(self.rf_plot)
        self._widget.switch_widget_rb(self.rb_plot)
        # Create QWidget
        #self._widget = QWidget()
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

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
        
        
    
    