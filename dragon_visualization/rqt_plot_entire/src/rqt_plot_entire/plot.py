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
from .rqt_plot_legs.src.rqt_plot_legs.plot import Plot as LegsPlot
from .rqt_plot_lf_foot.src.rqt_plot_lf_foot.plot import Plot as LfFootPlot
from .rqt_plot_lb_foot.src.rqt_plot_lf_foot.plot import Plot as LbFootPlot
from .rqt_plot_rf_foot.src.rqt_plot_lf_foot.plot import Plot as RfFootPlot
from .rqt_plot_rb_foot.src.rqt_plot_lf_foot.plot import Plot as RbFootPlot
from .rqt_plot_foots.src.rqt_plot_lf_foot.plot import  Plot as FootsPlot

class Plot(Plugin):

    def __init__(self, context):
        super(Plot, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('EntirePlot')

        self._widget = PlotWidget()
        self.lf_plot = LFPlot()
        self.lb_plot = LBPlot()
        self.rf_plot = RFPlot()
        self.rb_plot = RBPlot()
        self.legs_plot = LegsPlot()
        self.lf_foot_plot = LfFootPlot()
        self.lb_foot_plot = LbFootPlot()
        self.rf_foot_plot = RfFootPlot()
        self.rb_foot_plot = RbFootPlot()
        self.foots_plot = FootsPlot()
        self._widget.switch_widget_lf(self.lf_plot)
        self._widget.switch_widget_lb(self.lb_plot)
        self._widget.switch_widget_rf(self.rf_plot)
        self._widget.switch_widget_rb(self.rb_plot)
        self._widget.switch_widget_legs(self.legs_plot)
        self._widget.switch_widget_lf_foot(self.lf_foot_plot)
        self._widget.switch_widget_lb_foot(self.lb_foot_plot)
        self._widget.switch_widget_rf_foot(self.rf_foot_plot)
        self._widget.switch_widget_rb_foot(self.rb_foot_plot)
        self._widget.switch_widget_foots(self.foots_plot)
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
        
        
    
    