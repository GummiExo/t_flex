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
        self.variable_stiffness_angle1 = 3355
        self.variable_stiffness_angle2 = 3190
        self.Motor1State = None
        self.Motor2State = None
        rospy.init_node('ankle_controller', anonymous = True)
        rospy.Subscriber("/motor_states/pan_tilt_port",MotorStateList, self.updateMotorState)
        rospy.Subscriber("/flag_stiffness_calibration", Bool, self.updateFlagStiffnessCalibration)
        self.isMotorAngleUpdated = False
        self.CALIBRATE = True

    def updateMotorState(self, data):
        if len(data.motor_states) < 2:
            self.isMotorAngleUpdated = False
        else:
            self.Motor2State = data.motor_states[1]
            self.Motor1State = data.motor_states[0]
            self.isMotorAngleUpdated = True

    def updateFlagStiffnessCalibration(self, data):
        self.CALIBRATE = data.data


    def calibration_variable_stiffness(self):
        rospy.loginfo("Calibration Thread Started")
        self.CALIBRATE = True
        while self.CALIBRATE:
            if self.isMotorAngleUpdated == True:
				if self.Motor1State.position < self.variable_stiffness_angle1:
					self.variable_stiffness_angle1  = self.Motor1State.position
				if self.Motor2State.position < self.variable_stiffness_angle2:
					self.variable_stiffness_angle2  = self.Motor2State.position
				self.isMotorAngleUpdated = False
        rospy.loginfo("Calibration Thread Finished")

    def process(self):
        m1 = (0+3.28)/(3354-1218)
        m2 = (3.28-0)/(3187-1063)
        self.StiffnessValueToPub1 = m1*(self.variable_stiffness_angle1- 3354)
        self.StiffnessValueToPub2 = m2*(self.variable_stiffness_angle2- 1063)
        # Validation Motor id 3
        if self.StiffnessValueToPub1 > 0:
            self.StiffnessValueToPub1 = 0
        if self.StiffnessValueToPub1 < -3.28:
            self.StiffnessValueToPub1 = -3.28
        # Validation Motor id 4
        if self.StiffnessValueToPub2 > 3.28:
            self.StiffnessValueToPub2 = 3.28
        if self.StiffnessValueToPub2 < 0:
            self.StiffnessValueToPub2 = 0
        rospy.loginfo("Stiffness Value motor 1 = %s Stiffness Value motor 2 = %s",self.StiffnessValueToPub1,self.StiffnessValueToPub2)
        rospy.loginfo("Position in Stiffness motor 1 = %s Position in Stiffness motor 2 = %s",self.variable_stiffness_angle1,self.variable_stiffness_angle2)
        home = os.path.expanduser("~")
        os.chdir(home + '/catkin_ws/src/gummi_ankle/yaml')
        f = open('calibrationStiffness.yaml','w+')
        info = ["StiffnessValueToPub1: "+str(self.StiffnessValueToPub1),
                "StiffnessValueToPub2: "+str(self.StiffnessValueToPub2)]
        f.write("\n".join(info))
        f.close()

def main():

    c = T_FlexCalibration()
    rate = rospy.Rate(10)
    while not (rospy.is_shutdown()):
        if not (c.Motor1State == None and c.Motor2State == None):
            c.calibration_variable_stiffness()
            break
    rospy.on_shutdown(c.process)
    rospy.loginfo("Calibration Finished")

if __name__ == '__main__':
    main()
