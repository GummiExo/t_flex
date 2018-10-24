#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
import os
from dynamixel_msgs.msg import MotorStateList
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64, Bool

class T_FlexCalibration(object):
    def __init__(self):
        self.max_angle_motor1 = 1177
        self.min_angle_motor1 = 3355
        self.max_angle_motor2 = 963
        self.min_angle_motor2 = 3190
        self.Motor1State = None
        self.Motor2State = None
        rospy.init_node('ankle_controller', anonymous = True)
        rospy.Subscriber("/motor_states/pan_tilt_port",MotorStateList, self.updateMotorState)
        rospy.Subscriber("/flag_angle_calibration", Bool, self.updateFlagAngleCalibration)
        self.isMotorAngleUpdated = False
        self.CALIBRATE = True

    def updateMotorState(self, data):
        if len(data.motor_states) < 2:
            self.isMotorAngleUpdated = False
        else:
            self.Motor2State = data.motor_states[1]
            self.Motor1State = data.motor_states[0]
            self.isMotorAngleUpdated = True

    def updateFlagAngleCalibration(self, data):
        self.CALIBRATE = data.data

    def calibration_auto_movement(self):
        rospy.loginfo("Calibration Thread Started")
        self.CALIBRATE = True
        while self.CALIBRATE:
            if self.isMotorAngleUpdated == True:
                if self.Motor1State.position > self.max_angle_motor1:
                    self.max_angle_motor1 = self.Motor1State.position
                if self.Motor1State.position < self.min_angle_motor1:
                    self.min_angle_motor1 = self.Motor1State.position
                if self.Motor2State.position > self.max_angle_motor2:
                    self.max_angle_motor2 = self.Motor2State.position
                if self.Motor2State.position < self.min_angle_motor2:
                    self.min_angle_motor2 = self.Motor2State.position
                self.isMotorAngleUpdated = False
        rospy.loginfo("Calibration Thread Finished")

    def process(self):
        m1 = (0+3.28)/(3354-1218)
        m2 = (3.28-0)/(3187-1063)
        self.ValueToPubUp1= m1*(self.min_angle_motor1 - 3354)
        self.ValueToPubDown1= m1*(self.max_angle_motor1- 3354)
        rospy.loginfo("Value Up motor 1 = %s Value Down motor 1 = %s",self.ValueToPubUp1,self.ValueToPubDown1)
        self.ValueToPubUp2 = m2*(self.min_angle_motor2- 1063)
        self.ValueToPubDown2= m2*(self.max_angle_motor2- 1063)
        # Validation Motor id 3
        if self.ValueToPubUp1 > 0:
            self.ValueToPubUp1 = 0
        if self.ValueToPubUp1 < -3.28:
            self.ValueToPubUp1 = -3.28
        if self.ValueToPubDown1 > 0:
            self.ValueToPubDown1 = 0
        if self.ValueToPubDown1 < -3.28:
            self.ValueToPubDown1 = -3.28
        # Validation Motor id 4
        if self.ValueToPubUp2 > 3.28:
            self.ValueToPubUp2 = 3.28
        if self.ValueToPubUp2 < 0:
            self.ValueToPubUp2 = 0
        if self.ValueToPubDown2 > 3.28:
            self.ValueToPubDown2 = 3.28
        if self.ValueToPubDown2 < 0:
            self.ValueToPubDown2 = 0
        rospy.loginfo("Value Up motor 2 = %s Value Down motor 2 = %s",self.ValueToPubUp2,self.ValueToPubDown2)
        rospy.loginfo("Position max motor 1 = %s Position min motor 1 = %s",self.max_angle_motor1,self.min_angle_motor1)
        rospy.loginfo("Position max motor 2 = %s Position min motor 2 = %s",self.max_angle_motor2,self.min_angle_motor2)
        home = os.path.expanduser("~")
        os.chdir(home + '/catkin_ws/src/gummi_ankle/yaml')
        f = open('calibrationAngle.yaml','w+')
        info = ["ValueToPubUp1: "+str(self.ValueToPubUp1),
                "ValueToPubDown1: "+str(self.ValueToPubDown1),
                "ValueToPubUp2: "+str(self.ValueToPubUp2),
                "ValueToPubDown2: "+str(self.ValueToPubDown2)]
        f.write("\n".join(info))
        f.close()

def main():

    c = T_FlexCalibration()
    rate = rospy.Rate(10)
    while not (rospy.is_shutdown()):
        if not (c.Motor1State == None and c.Motor2State == None):
            c.calibration_auto_movement()
            break
    rospy.on_shutdown(c.process)
    rospy.loginfo("Calibration Finished")

if __name__ == '__main__':
    main()
