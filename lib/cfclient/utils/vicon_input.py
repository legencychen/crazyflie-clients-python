__author__ = 'ChenYing'
__all__ = ['ViconReader']

import sys
import os
import re
import glob
import traceback
import logging
import shutil
import vrpn
logger = logging.getLogger(__name__)
import socket
from cfclient.utils.periodictimer import PeriodicTimer
from cflib.utils.callbacks import Caller
import time
class ViconReader:
    """
    Thread will read the position feedback and quarternion from vicon system, and send it to the trajectory planner.
    And the trajectory planner will plan the reference signal and send them to crazyflie
    """

    def __init__(self):

        self.input_updated = Caller()
        #self.start_time = []
        #self.old_start_time = time.time()


        #self.vicon_tracker = vrpn.receiver.Tracker("nothing@vicon")
        #self.vicon_tracker.register_change_handler("position",self.vicon_tracker_callback,"position")

        #self.vicon_check_timer = PeriodicTimer(1.0, self.vicon_check)
        #self.vicon_check_timer.start()
        self.vicon_read_timer = PeriodicTimer(0.01, self.vicon_read)
        self.vicon_connection_check_timer = PeriodicTimer(1.0, self.vicon_connection_check)
        self.vicon_connection_check_timer.start()

        self.vicon_name = []


    def vicon_connection_check(self):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection_result = self.sock.connect_ex(('192.168.1.1',80))

        if self.connection_result == 0:   # need to add extra try...exception... in case the vicon name is not right
            self.set_vicon_tracker(self.vicon_name)
            self.vicon_read_timer.start()
            self.vicon_connection_check_timer.stop()
            logger.info("port is connected")

        #else:

            #logger.info(" port not connected  %s", self.vicon_name)



    def set_vicon_tracker(self,name):

        tracker_name = str(name) + '@vicon'
        self.vicon_tracker = vrpn.receiver.Tracker(str(tracker_name))
        self.vicon_tracker.register_change_handler("position",self.vicon_tracker_callback,"position")


    def vicon_tracker_callback(self,userdata, data):

        self.input_updated.call(data['position'][0], data['position'][1], data['position'][2] , data['quaternion'][3], data['quaternion'][0], data['quaternion'][1], data['quaternion'][2])


    def vicon_read(self):

        self.vicon_tracker.mainloop()
        #self.start_time = time.time()
        #logger.info("vicon_read works")
        #logger.info("Vicon_read_timer interval:  %s", self.start_time - self.old_start_time)
        #self.old_start_time = self.start_time*1.0





