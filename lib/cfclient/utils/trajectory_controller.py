__author__ = 'ChenYing'
__all__ = ['Trajectory_Controller']

import logging
logger = logging.getLogger(__name__)
import time

import numpy as np
import numpy.matlib
from cflib.utils.callbacks import Caller
from copy import *

MAX_THRUST = 65000.0
MASS_THRUST = 42000.0

class Trajectory_Controller:
    """
    Upper controller to control the position of crazyflie in space with help of Vicon system.
    This class receives the position and quaternion signals and calculated the reference signal, thrust, yaw, pitch,roll angles.
    """

    def __init__(self):

        self.MAX_ANGLE = np.pi/4

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.current_position = np.array([0.0, 0.0, 0.0])
        #self.target_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.current_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.output_thrust = 0.0
        self.output = np.array([0.0, 0.0, 0.0, 0.0])
        self.output_update = Caller()
        self.old_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])

        self.kp = 12000.0
        self.kd = 20000.0
        self.ki = 1000.0
        self.emergency_stop = True
        self.controller_start = False
        self.writing_start = False
        #self.time_start = []
        #self.time_end = []

        self.angle = 0.0
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        self.current_z_axis = np.array([0.0, 0.0, 1.0])
        self.current_r_error_integration = np.array([0.0, 0.0, 0.0])

        self.current_r_error = np.array([0.0, 0.0, 0.1])
        self.current_r_error_unit = np.array([0.0, 0.0, 0.1])

        self.recording = np.matlib.zeros((10000, 10))
        self.recording_i = 0

    def write_data(self,zd_x,zd_y,zd_z):

        if self.recording_i < 10000:

            self.recording[self.recording_i,0] = self.current_r_error[0]*1.0
            self.recording[self.recording_i,1] = self.current_r_error[1]*1.0
            self.recording[self.recording_i,2] = self.current_r_error[2]*1.0
            self.recording[self.recording_i,3] = zd_x*1.0
            self.recording[self.recording_i,4] = zd_y*1.0
            self.recording[self.recording_i,5] = zd_z*1.0
            self.recording[self.recording_i,6] = self.current_z_axis[0]*1.0
            self.recording[self.recording_i,7] = self.current_z_axis[1]*1.0
            self.recording[self.recording_i,8] = self.current_z_axis[2]*1.0
            self.recording[self.recording_i,9] = self.output_thrust*1.0

            self.recording_i += 1
        else:

            logger.info("recording matrix is full")

    def save_data(self):

        path = '/Users/ChenYing/Research/research/M-pc-client_vicon/vicon_data'
        name = str(int(time.time()))
        np.savetxt(str(path) +'/' +  name + '.csv',self.recording, delimiter =',')

        self.recording = np.matlib.zeros((10000, 10))
        self.recording_i = 0


    def control_gain_recover(self):

        self.kp = 12000.0
        self.kd = 20000.0
        self.ki = 1000.0

    def set_emergency_stop(self):
        self.emergency_stop = True

    def recover_emergency_stop(self):
        self.emergency_stop = False

    def start_controller(self):
        self.controller_start = True



    def set_target_position(self,x,y,z):

        self.target_position[0] = x
        self.target_position[1] = y
        self.target_position[2] = z


    def vicon_receive(self,x,y,z,q0,q1,q2,q3):

        #self.time_start = time.time()

        self.current_position[0] = x
        self.current_position[1] = y
        self.current_position[2] = z

        self.current_velocity = (self.current_position - self.old_position)/0.01

        self.old_position[0] = x
        self.old_position[1] = y
        self.old_position[2] = z
        # self.old_position = self.current_position

        self.current_quaternion[0] = q0
        self.current_quaternion[1] = q1
        self.current_quaternion[2] = q2
        self.current_quaternion[3] = q3

        self.current_z_axis = np.array([2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), q0*q0+q3*q3-q1*q1-q2*q2])
        self.current_z_axis = self.current_z_axis/np.linalg.norm(self.current_z_axis)

        self.current_r_error = self.target_position - self.current_position
        self.current_r_error_unit = self.current_r_error/np.linalg.norm(self.current_r_error)

        #logger.info("Position error: %s  %s  %s", self.current_r_error[0], self.current_r_error[1], self.current_r_error[2])
        logger.info("body z axis %s %s %s",2*q1*q3 + 2*q0*q2,2*q2*q3 - 2*q0*q1,q0*q0 - q1*q1 - q2*q2 + q3*q3)

        self.current_r_error_integration += self.current_r_error*0.01

        if np.linalg.norm(self.current_r_error_integration) >= 6:

            self.current_r_error_integration = 6.0*self.current_r_error_integration/np.linalg.norm(self.current_r_error_integration)

        if self.controller_start:

            self.controller()

    def reset_integration_controller(self):

        self.current_r_error_integration = np.array([0.0, 0.0, 0.0])
        logger.info("reset_integration_controller")


    def controller(self):

        z_axis_desired = np.array([0.0,0.0,0.0])

        r_error_norm = np.linalg.norm(self.current_r_error)

        if r_error_norm >= 5.0:

            self.target_velocity = 1.3*self.current_r_error_unit  # The velocity in the target position direction is 1 m/s

        else:

            self.target_velocity = 1.3*(r_error_norm/5.0)*self.current_r_error_unit

        z_axis_desired = self.parameter_decrease() # Find the appropriated thrust force in order to limit it in a range angle
        z_axis_desired_unit = z_axis_desired/np.linalg.norm(z_axis_desired)

        #logger.info("Current_z_axis X is %s Y is %s Z is %s", self.current_z_axis[0], self.current_z_axis[1], self.current_z_axis[2])
        self.output_thrust = np.dot(z_axis_desired, self.current_z_axis)

        if self.output_thrust < 0:
            self.output_thrust = 0.0

        if self.output_thrust > MAX_THRUST:
            self.output_thrust = deepcopy(MAX_THRUST)

        x_axis_desired = np.cross(np.array([0.0, 1.0, 0.0]), z_axis_desired_unit)
        x_axis_desired = x_axis_desired/np.linalg.norm(x_axis_desired)
        y_axis_deisred = np.cross(z_axis_desired_unit, x_axis_desired)

        pitch_angle = np.arcsin(-1.0*x_axis_desired[2])*180.0/np.pi
        yaw_angle = np.arctan2(x_axis_desired[1], x_axis_desired[0])*180.0/np.pi
        roll_angle = np.arctan2(y_axis_deisred[2], z_axis_desired_unit[2])*180.0/np.pi

        #logger.info("%f, %f, %f, %f, %f,%f,%f", self.output_thrust, roll_angle, pitch_angle, yaw_angle, self.current_z_axis[0],self.current_z_axis[1],self.current_z_axis[2])

        if self.emergency_stop:

            self.output_update.call(0.0, 0.0, 0.0, 0.0)
            self.controller_start = False
            self.current_r_error_integration *= 0.0



        else:
            #logger.info("Output_update  thrust is %s  pos_X is %s  pos_Y is %s  pos_Z is %s Yaw is %s  Pitch is %s  Roll is %s",
                        #self.output_thrust, self.current_position[0], self.current_position[1], self.current_position[2], yaw_angle, pitch_angle, roll_angle)
            self.output_update.call(roll_angle, pitch_angle, yaw_angle,self.output_thrust)
            #self.time_end = time.time()
            #logger.info("Time spent in one round:  %s", self.time_end - self.time_start)

            if self.writing_start:
                self.write_data(z_axis_desired[0], z_axis_desired[1],z_axis_desired[2])



    def parameter_decrease(self):

        temp_z_desired = MASS_THRUST*np.array([0.0, 0.0, 1.0]) + self.kp*self.current_r_error + self.kd*(self.target_velocity - self.current_velocity) + self.ki*self.current_r_error_integration

        #logger.info("Target_velocity  V_X is %s  V_Y is %s  V_Z is %s", self.target_velocity[0], self.target_velocity[1], self.target_velocity[2])
        #logger.info("Current_velocity  V_X is %s  V_Y is %s  V_Z is %s", self.current_velocity[0], self.current_velocity[1], self.current_velocity[2])
        #logger.info("Velocity_error V_X is %s  V_Y is %s  V_Z is %s", self.target_velocity[0] - self.current_velocity[0],self.target_velocity[1] - self.current_velocity[1], self.target_velocity[2] - self.current_velocity[2])

        self.angle = np.arccos(np.dot(temp_z_desired/np.linalg.norm(temp_z_desired), np.array([0.0, 0.0, 1.0]) ))

        #logger.info("Angle is %s  Magnitude of thrust is %s", self.angle*180.0/np.pi, np.linalg.norm(temp_z_desired))

        logger.info("Angle is %s", self.angle*180/np.pi)

        while self.angle >= self.MAX_ANGLE:

            self.kp *= 0.9
            self.kd *= 0.9
            self.ki *= 0.9

            temp_z_desired = MASS_THRUST*np.array([0.0, 0.0, 1.0]) + self.kp*self.current_r_error + self.kd*(self.target_velocity - self.current_velocity) + self.ki*self.current_r_error_integration
            self.angle = np.arccos(np.dot(temp_z_desired/np.linalg.norm(temp_z_desired), np.array([0.0, 0.0, 1.0]) ))

        self.control_gain_recover()

        logger.info("P control: %s  D control: %s  I control: %s", np.linalg.norm(self.kp*self.current_r_error),  np.linalg.norm(self.kd*(self.target_velocity - self.current_velocity)), np.linalg.norm(self.ki*self.current_r_error_integration) )

        return temp_z_desired






































