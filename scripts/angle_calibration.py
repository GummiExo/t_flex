#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
import os
from dynamixel_workbench_msgs.msg import DynamixelStateList
from std_msgs.msg import Bool

class T_FlexCalibration(object):
    def __init__(self):
        self.max_angle_motor1 = 900
        self.min_angle_motor1 = 3000
        self.max_angle_motor2 = 1200
        self.min_angle_motor2 = 3300
        self.Motor1State = None
        self.Motor2State = None
        rospy.init_node('angle_calibration', anonymous = True)
        rospy.Subscriber("/dynamixel_state",DynamixelStateList, self.updateMotorState)
        rospy.Subscriber("/kill_angle_calibration", Bool, self.updateFlagAngleCalibration)
        self.isMotorAngleUpdated = False
        self.CALIBRATE = False

    def updateMotorState(self, motor_info):
        if len(motor_info.dynamixel_state) < 2:
            self.isMotorAngleUpdated = False
        else:
            self.Motor1State = motor_info.dynamixel_state[0]
            self.Motor2State = motor_info.dynamixel_state[1]
            self.isMotorAngleUpdated = True

    def updateFlagAngleCalibration(self, flag):
        self.CALIBRATE = flag.data

    def calibration_auto_movement(self):
        rospy.loginfo("Calibration Thread Started")
        while not self.CALIBRATE:
            if self.isMotorAngleUpdated == True:
                if self.Motor1State.present_position > self.max_angle_motor1:
                    self.max_angle_motor1 = self.Motor1State.present_position
                if self.Motor1State.present_position < self.min_angle_motor1:
                    self.min_angle_motor1 = self.Motor1State.present_position
                if self.Motor2State.present_position > self.max_angle_motor2:
                    self.max_angle_motor2 = self.Motor2State.present_position
                if self.Motor2State.present_position < self.min_angle_motor2:
                    self.min_angle_motor2 = self.Motor2State.present_position
                self.isMotorAngleUpdated = False
        rospy.loginfo("Calibration Thread Finished")

    def process(self):
        self.ValueToPubUp1= self.min_angle_motor1
        self.ValueToPubDown1= self.max_angle_motor1
        self.ValueToPubUp2 = self.min_angle_motor2
        self.ValueToPubDown2= self.max_angle_motor2
        # Validation Motor id 3
        if self.ValueToPubUp1 > 2900:
            self.ValueToPubUp1 = 2900
        if self.ValueToPubUp1 < 1000:
            self.ValueToPubUp1 = 1000
        if self.ValueToPubDown1 > 2900:
            self.ValueToPubDown1 = 2900
        if self.ValueToPubDown1 < 1000:
            self.ValueToPubDown1 = 2900
        # Validation Motor id 4
        if self.ValueToPubUp2 > 3200:
            self.ValueToPubUp2 = 3200
        if self.ValueToPubUp2 < 1300:
            self.ValueToPubUp2 = 1300
        if self.ValueToPubDown2 > 3200:
            self.ValueToPubDown2 = 3200
        if self.ValueToPubDown2 < 1300:
            self.ValueToPubDown2 = 1300
        rospy.loginfo("Value Up motor 1 = %s Value Down motor 1 = %s",self.ValueToPubUp1,self.ValueToPubDown1)
        rospy.loginfo("Value Up motor 2 = %s Value Down motor 2 = %s",self.ValueToPubUp2,self.ValueToPubDown2)
        home = os.path.expanduser("~")
        os.chdir(home + '/catkin_ws/src/t_flex/yaml')
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
        if not (c.Motor1State == None and c.Motor2State == None):
            c.calibration_auto_movement()
            break
    rospy.on_shutdown(c.process)
    rospy.loginfo("Calibration Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print (e)
