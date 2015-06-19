__author__ = 'ChenYing'
__all__ = ['LogTab']

import time
import sys
import logging
logger = logging.getLogger(__name__)

from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtCore import Qt, pyqtSlot, pyqtSignal, QThread, SIGNAL

from cflib.crazyflie import Crazyflie

from cfclient.ui.tab import Tab

vicon_tab_class = uic.loadUiType(sys.path[0] +
                                 "/cfclient/ui/tabs/viconTab.ui")[0]

class ViconTab(Tab,vicon_tab_class):

    _vicon_data_signal = pyqtSignal(float,float,float,float,float,float,float)
    _flipstart_signal = pyqtSignal(bool)


    def __init__(self, tabWidget, helper, *args):

        super(ViconTab, self).__init__(*args)
        self.setupUi(self)

        self.tabName = "Vicon Read"
        self.menuName = "Vicon Read"

        self.tabWidget = tabWidget
        self.helper = helper

        self._vicon_data_signal.connect(self.update_input)
        self.helper.vicon_reader.input_updated.add_callback(self._vicon_data_signal.emit)

        self.vicon_name.returnPressed.connect(self.update_vicon_name)
        self.x_target.setRange(-5.0, 5.0)
        self.x_target.setSingleStep(0.1)
        self.y_target.setRange(-5.0, 5.0)
        self.y_target.setSingleStep(0.1)
        self.z_target.setRange(0.0, 5.0)
        self.z_target.setSingleStep(0.1)

        self.emergency_stop.clicked.connect(lambda enabled:    self.helper.trajectoryController.set_emergency_stop())
        self.emergency_recover.clicked.connect(lambda enabled: self.helper.trajectoryController.recover_emergency_stop())
        self.controller_start.clicked.connect(lambda enabled:  self.helper.trajectoryController.start_controller())
        self.save_data.clicked.connect(lambda enabled:  self.helper.trajectoryController.save_data())
        self.flip_start.clicked.connect(self._flipstart_signal.emit)
        self._flipstart_signal.connect(self.flipstart_function)


        self.start_write.stateChanged.connect(self.data_writing)

        self.x_target.valueChanged.connect(self.update_target_position)
        self.y_target.valueChanged.connect(self.update_target_position)
        self.z_target.valueChanged.connect(self.update_target_position)

        self.x_target.valueChanged.connect(self.reset_i_controller)
        self.y_target.valueChanged.connect(self.reset_i_controller)
        self.z_target.valueChanged.connect(self.reset_i_controller)

    def flipstart_function(self):

        self.helper.cf.param.set_value("alticontroller.flipstart", str(1))
        logger.info("flip start received ")



    def data_writing(self,state):

        if state == Qt.Checked:

            self.helper.trajectoryController.writing_start = True
            logger.info("checked")
            logger.info("the sign is %s", self.helper.trajectoryController.writing_start )

        else:

            self.helper.trajectoryController.writing_start = False
            logger.info("not checked")
            logger.info("the sign is %s", self.helper.trajectoryController.writing_start)

    def update_input(self,x,y,z,q0,q1,q2,q3):


        self.x_position.setText(("%f" % x))
        self.y_position.setText(("%f" % y))
        self.z_position.setText(("%f" % z))
        current_z_axis_1 = 2*(q1*q3+q0*q2)
        current_z_axis_2 = 2*(q2*q3-q0*q1)
        current_z_axis_3 = q0*q0+q3*q3-q1*q1-q2*q2
        self.q0.setText(("%f" % current_z_axis_1))
        self.q1.setText(("%f" % current_z_axis_2))
        self.q2.setText(("%f" % current_z_axis_3))
        self.q3.setText(("%f" % q3))


    def update_vicon_name(self):


        name = str(self.vicon_name.displayText()) + '@vicon'
        logger.info("new name %s ", name)
        self.helper.vicon_reader.set_vicon_tracker(str(name))

    def update_target_position(self):

        self.helper.trajectoryController.set_target_position( self.x_target.value(), self.y_target.value(), self.z_target.value()  )
        logger.info("new target %s  %s %s", self.x_target.value(), self.y_target.value(), self.z_target.value())

    def reset_i_controller(self):

        self.helper.trajectoryController.reset_integration_controller()
        logger.info("reset i controller")



