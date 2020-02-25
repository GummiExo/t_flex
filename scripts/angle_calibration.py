#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
import os
import math
from dynamixel_msgs.msg import MotorStateList
import rosparam, rospkg
from std_msgs.msg import Bool

class T_FlexCalibration(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_directory = rospack.get_path('t_flex')
        params = rosparam.load_file(self.package_directory + '/yaml/motors_parameters.yaml')
        self.max_value_motor1 = params[0][0]['tilt1_controller']['motor']['max']
        self.min_value_motor1 = params[0][0]['tilt1_controller']['motor']['min']
        self.init_value_motor1 = params[0][0]['tilt1_controller']['motor']['init']
        self.max_value_motor2 = params[0][0]['tilt2_controller']['motor']['max']
        self.min_value_motor2 = params[0][0]['tilt2_controller']['motor']['min']
        self.init_value_motor2 = params[0][0]['tilt2_controller']['motor']['init']
        self.max_angle_motor1 = self.min_value_motor1
        self.min_angle_motor1 = self.max_value_motor1
        self.max_angle_motor2 = self.min_value_motor2
        self.min_angle_motor2 = self.max_value_motor2
        self.MotorStateFrontal = None
        self.MotorStatePosterior = None
        rospy.init_node('t_flex_angle_calibration', anonymous = True)
        rospy.Subscriber("/motor_states/frontal_tilt_port",MotorStateList, self.updateMotorStateFrontal)
        #rospy.Subscriber("/motor_states/posterior_tilt_port",MotorStateList, self.updateMotorStatePosterior)
        rospy.Subscriber("/t_flex/kill_angle_calibration", Bool, self.updateFlagAngleCalibration)
        self.isMotorAngleUpdatedFrontal = False
        self.isMotorAngleUpdatedPosterior = False
        self.CALIBRATE = False

    def updateMotorStateFrontal(self, motor_info):
	try:
        self.MotorStateFrontal = motor_info.motor_states[0]
	    self.MotorStatePosterior = motor_info.motor_states[1]
        self.isMotorAngleUpdatedFrontal = True
	    self.isMotorAngleUpdatedPosterior = True
	except:
	    self.isMotorAngleUpdatedFrontal = False

    def updateMotorStatePosterior(self, motor_info):
        self.MotorStatePosterior = motor_info.motor_states[0]
        self.isMotorAngleUpdatedPosterior = True

    def updateFlagAngleCalibration(self, flag):
        self.CALIBRATE = flag.data

    def calibration_auto_movement(self):
        rospy.loginfo("Calibration Thread Started")
        while not self.CALIBRATE:
            if self.isMotorAngleUpdatedFrontal == True:
                if self.MotorStateFrontal.position > self.max_angle_motor1:
                    self.max_angle_motor1 = self.MotorStateFrontal.position
                if self.MotorStateFrontal.position < self.min_angle_motor1:
                    self.min_angle_motor1 = self.MotorStateFrontal.position
                    self.isMotorAngleUpdatedFrontal = False
            if self.isMotorAngleUpdatedPosterior == True:
                if self.MotorStatePosterior.position > self.max_angle_motor2:
                    self.max_angle_motor2 = self.MotorStatePosterior.position
                if self.MotorStatePosterior.position < self.min_angle_motor2:
                    self.min_angle_motor2 = self.MotorStatePosterior.position
                self.isMotorAngleUpdatedPosterior = False
        rospy.loginfo("Calibration Thread Finished")

    def pos_to_rad(self,position,initial):
        rad = (position - initial)*2*math.pi/4095
        return rad

    def process(self):
        ''' Motor Frontal '''
        if self.init_value_motor1 == self.min_value_motor1:
            self.ValueToPubUp1 = self.max_angle_motor1
            self.ValueToPubDown1= self.min_angle_motor1
        else:
            self.ValueToPubUp1 = self.min_angle_motor1
            self.ValueToPubDown1 = self.max_angle_motor1
        ''' Motor Posterior '''
        if self.init_value_motor2 == self.min_value_motor2:
            self.ValueToPubUp2 = self.max_angle_motor2
            self.ValueToPubDown2 = self.min_angle_motor2
        else:
            self.ValueToPubUp2 = self.min_angle_motor2
            self.ValueToPubDown2 = self.max_angle_motor2
        ''' Validation of Range '''
        # Validation Motor id 1
        if self.ValueToPubUp1 > self.max_value_motor1:
            self.ValueToPubUp1 = self.max_value_motor1
        if self.ValueToPubUp1 < self.min_value_motor1:
            self.ValueToPubUp1 = self.min_value_motor1
        if self.ValueToPubDown1 > self.max_value_motor1:
            self.ValueToPubDown1 = self.max_value_motor1
        if self.ValueToPubDown1 < self.min_value_motor1:
            self.ValueToPubDown1 = self.min_value_motor1
        # Validation Motor id 2
        if self.ValueToPubUp2 > self.max_value_motor2:
            self.ValueToPubUp2 = self.max_value_motor2
        if self.ValueToPubUp2 < self.min_value_motor2:
            self.ValueToPubUp2 = self.min_value_motor2
        if self.ValueToPubDown2 > self.max_value_motor2:
            self.ValueToPubDown2 = self.max_value_motor2
        if self.ValueToPubDown2 < self.min_value_motor2:
            self.ValueToPubDown2 = self.min_value_motor2
        rospy.loginfo("Value Up motor 1 = %s Value Down motor 1 = %s",self.ValueToPubUp1,self.ValueToPubDown1)
        rospy.loginfo("Value Up motor 2 = %s Value Down motor 2 = %s",self.ValueToPubUp2,self.ValueToPubDown2)
        ''' Conversion to Randians '''
        self.ValueToPubUp1 = self.pos_to_rad(self.ValueToPubUp1,self.init_value_motor1)
        self.ValueToPubDown1 = self.pos_to_rad(self.ValueToPubDown1,self.init_value_motor1)
        self.ValueToPubUp2 = self.pos_to_rad(self.ValueToPubUp2,self.init_value_motor2)
        self.ValueToPubDown2 = self.pos_to_rad(self.ValueToPubDown2,self.init_value_motor2)
        rospy.loginfo("Value Up motor 1 Radians = %s Value Down motor 1 Radians = %s",self.ValueToPubUp1,self.ValueToPubDown1)
        rospy.loginfo("Value Up motor 2 Radians = %s Value Down motor 2 Radians = %s",self.ValueToPubUp2,self.ValueToPubDown2)
        os.chdir(self.package_directory + '/yaml')
        f = open('calibrationAngle.yaml','w+')
        info = ["ValueToPubUp1: "+str(self.ValueToPubUp1),
                "ValueToPubDown1: "+str(self.ValueToPubDown1),
                "ValueToPubUp2: "+str(self.ValueToPubUp2),
                "ValueToPubDown2: "+str(self.ValueToPubDown2)]
        f.write("\n".join(info))
        f.close()

def main():

    c = T_FlexCalibration()
    rate = rospy.Rate(50)
    while not (rospy.is_shutdown()):
        if not (c.MotorStateFrontal == None and c.MotorStatePosterior == None):
            c.calibration_auto_movement()
            break
    rospy.on_shutdown(c.process)
    rospy.loginfo("Calibration Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print (e)
