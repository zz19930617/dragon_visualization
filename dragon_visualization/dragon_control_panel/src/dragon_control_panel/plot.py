import argparse

from python_qt_binding import QT_BINDING
from python_qt_binding.QtCore import qDebug
from rqt_gui_py.plugin import Plugin

from rqt_py_common.ini_helper import pack, unpack

from .plot_widget import PlotWidget

from .data_plot import DataPlot

class Plot(Plugin):
    
    def __init__(self , context):
        super(Plot , self).__init__(context)
        self.setObjectName('plot')
        
        self._context = context
        self._widget = PlotWidget()
        self._data_plot = DataPlot(self._widget)
        
        #set parameters of data_plot 
        self._data_plot.set_autoscale(x=False)
        self._data_plot.set_autoscale(y=DataPlot.SCALE_EXTEND | DataPlot.SCALE_VISIBLE)
        self._data_plot.set_xlim([0 , 10.0])
        
        self._widget.switch_data_plot_widget(self._data_plot)
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        
    def _update_title(self):
        #self._widget.setWindowTitle(self._data_plot.getTitle())
        if self._context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self._context.serial_number())) 
    
    #def save_settings(self, plugin_settings, instance_settings):
        #self._data_plot.save_settings(plugin_settings, instance_settings)
        
    #def restore_settings(self, plugin_settings, instance_settings):
        #self._update_title()
        #self._data_plot.restore_settings(plugin_settings, instance_settings)      
        
    #def trigger_configuration(self):
        #self._data_plot.doSettingsDialog()
        #self._update_title()

    def shutdown_plugin(self):
        self._widget.clean_up_subscribers()        
        
        
    
    