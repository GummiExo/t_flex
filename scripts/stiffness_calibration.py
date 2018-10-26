#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
import os
from dynamixel_workbench_msgs.msg import DynamixelStateList
from std_msgs.msg import Bool

class T_FlexCalibration(object):
    def __init__(self):
        self.variable_stiffness_angle1 = 3355
        self.variable_stiffness_angle2 = 3190
        self.Motor1State = None
        self.Motor2State = None
        rospy.init_node('stiffness_calibration', anonymous = True)
        rospy.Subscriber("/dynamixel_state",DynamixelStateList, self.updateMotorState)
        rospy.Subscriber("/kill_stiffness_calibration", Bool, self.updateFlagStiffnessCalibration)
        self.isMotorAngleUpdated = False
        self.CALIBRATE = False

    def updateMotorState(self, motor_info):
        if len(motor_info.dynamixel_state) < 2:
            self.isMotorAngleUpdated = False
        else:
            self.Motor1State = motor_info.dynamixel_state[0]
            self.Motor2State = motor_info.dynamixel_state[1]
            self.isMotorAngleUpdated = True

    def updateFlagStiffnessCalibration(self, flag):
        self.CALIBRATE = flag.data

    def calibration_variable_stiffness(self):
        rospy.loginfo("Calibration Thread Started")
        while not self.CALIBRATE:
            if self.isMotorAngleUpdated == True:
				if self.Motor1State.present_position < self.variable_stiffness_angle1:
					self.variable_stiffness_angle1  = self.Motor1State.present_position
				if self.Motor2State.present_position < self.variable_stiffness_angle2:
					self.variable_stiffness_angle2  = self.Motor2State.present_position
				self.isMotorAngleUpdated = False
        rospy.loginfo("Calibration Thread Finished")

    def process(self):
        self.StiffnessValueToPub1 = self.variable_stiffness_angle1
        self.StiffnessValueToPub2 = self.variable_stiffness_angle2
        # Validation Motor id 3
        # if self.StiffnessValueToPub1 > 0:
        #     self.StiffnessValueToPub1 = 0
        # if self.StiffnessValueToPub1 < -3.28:
        #     self.StiffnessValueToPub1 = -3.28
        # # Validation Motor id 4
        # if self.StiffnessValueToPub2 > 3.28:
        #     self.StiffnessValueToPub2 = 3.28
        # if self.StiffnessValueToPub2 < 0:
        #     self.StiffnessValueToPub2 = 0
        rospy.loginfo("Stiffness Value motor 1 = %s Stiffness Value motor 2 = %s",self.StiffnessValueToPub1,self.StiffnessValueToPub2)
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
        if not (c.Motor1State == None and c.Motor2State == None):
            c.calibration_variable_stiffness()
            break
    rospy.on_shutdown(c.process)
    rospy.loginfo("Calibration Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
