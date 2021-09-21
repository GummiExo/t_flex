#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
import os
import math
from dynamixel_msgs.msg import MotorStateList
import rosparam
from std_msgs.msg import Bool

class T_FlexCalibration(object):
    def __init__(self):
        home = os.path.expanduser('~')
        params = rosparam.load_file(home + '/catkin_ws/src/t_flex/yaml/motors_parameters.yaml')
        self.max_value_motor1 = params[0][0]['tilt1_controller']['motor']['max']
        self.min_value_motor1 = params[0][0]['tilt1_controller']['motor']['min']
        self.init_value_motor1 = params[0][0]['tilt1_controller']['motor']['init']
        self.max_value_motor2 = params[0][0]['tilt2_controller']['motor']['max']
        self.min_value_motor2 = params[0][0]['tilt2_controller']['motor']['min']
        self.init_value_motor2 = params[0][0]['tilt2_controller']['motor']['init']
        self.variable_stiffness_angle1 = self.init_value_motor1
        self.variable_stiffness_angle2 = self.init_value_motor2
        self.MotorStateFrontal = None
        self.MotorStatePosterior = None
        rospy.init_node('t_flex_stiffness_calibration', anonymous = True)
        rospy.Subscriber("/motor_states/frontal_tilt_port",MotorStateList, self.updateMotorStateFrontal)
        rospy.Subscriber("/motor_states/posterior_tilt_port",MotorStateList, self.updateMotorStatePosterior)
        rospy.Subscriber("/t_flex/kill_stiffness_calibration", Bool, self.updateFlagStiffnessCalibration)
        self.isMotorAngleUpdatedFrontal = False
        self.isMotorAngleUpdatedPosterior = False
        self.CALIBRATE = False

    def updateMotorStateFrontal(self, motor_info):
        self.MotorStateFrontal = motor_info.motor_states[0]
        self.isMotorAngleUpdatedFrontal = True

    def updateMotorStatePosterior(self, motor_info):
        self.MotorStatePosterior = motor_info.motor_states[0]
        self.isMotorAngleUpdatedPosterior = True

    def updateFlagStiffnessCalibration(self, flag):
        self.CALIBRATE = flag.data

    def calibration_variable_stiffness(self):
        rospy.loginfo("Calibration Thread Started")
        while not self.CALIBRATE:
            ''' Frontal Motor '''
            if self.isMotorAngleUpdatedFrontal == True:
                if self.init_value_motor1 == self.min_value_motor1:
    				if self.MotorStateFrontal.position > self.variable_stiffness_angle1:
    					self.variable_stiffness_angle1  = self.MotorStateFrontal.position
                else:
                    if self.MotorStateFrontal.position < self.variable_stiffness_angle1:
    					self.variable_stiffness_angle1  = self.MotorStateFrontal.position
                self.isMotorAngleUpdatedFrontal = False
            ''' Posterior '''
            if self.isMotorAngleUpdatedPosterior == True:
                if self.init_value_motor2 == self.min_value_motor2:
                    if self.init_value_motor2 == self.min_value_motor2:
        				if self.MotorStatePosterior.position > self.variable_stiffness_angle2:
        					self.variable_stiffness_angle2  = self.MotorStatePosterior.position
                    else:
                        if self.MotorStatePosterior.position < self.variable_stiffness_angle2:
        					self.variable_stiffness_angle2  = self.MotorStatePosterior.position
                self.isMotorAngleUpdatedPosterior = False
        rospy.loginfo("Calibration Thread Finished")

    def pos_to_rad(self,position,initial):
        rad = (position - initial)*2*math.pi/4095
        return rad

    def process(self):
        self.StiffnessValueToPub1 = self.variable_stiffness_angle1
        self.StiffnessValueToPub2 = self.variable_stiffness_angle2
        ''' Validation of Range '''
        # Validation Motor id 1
        if self.StiffnessValueToPub1 > self.max_value_motor1:
            self.StiffnessValueToPub1 = self.max_value_motor1
        if self.StiffnessValueToPub1 < self.min_value_motor1:
            self.StiffnessValueToPub1 = self.min_value_motor1
        # Validation Motor id 2
        if self.StiffnessValueToPub2 > self.max_value_motor2:
            self.StiffnessValueToPub2 = self.max_value_motor2
        if self.StiffnessValueToPub2 < self.min_value_motor2:
            self.StiffnessValueToPub2 = self.min_value_motor2
        rospy.loginfo("Stiffness Value motor 1 = %s Stiffness Value motor 2 = %s",self.StiffnessValueToPub1,self.StiffnessValueToPub2)
        ''' Conversion to Radians '''
        self.StiffnessValueToPub1 = self.pos_to_rad(self.StiffnessValueToPub1,self.init_value_motor1)
        self.StiffnessValueToPub2 = self.pos_to_rad(self.StiffnessValueToPub2,self.init_value_motor2)
        rospy.loginfo("Stiffness Value Motor 1 Radians = %s Stiffness Value Motor 2 Radians = %s",self.StiffnessValueToPub1,self.StiffnessValueToPub2)
        home = os.path.expanduser("~")
        os.chdir(home + '/catkin_ws/src/t_flex/yaml')
        f = open('calibrationStiffness.yaml','w+')
        info = ["StiffnessValueToPub1: "+str(self.StiffnessValueToPub1),
                "StiffnessValueToPub2: "+str(self.StiffnessValueToPub2)]
        f.write("\n".join(info))
        f.close()

def main():

    c = T_FlexCalibration()
    rate = rospy.Rate(10)
    while not (rospy.is_shutdown()):
        if not (c.MotorStateFrontal == None and c.MotorStatePosterior == None):
            c.calibration_variable_stiffness()
            break
    rospy.on_shutdown(c.process)
    rospy.loginfo("Calibration Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
