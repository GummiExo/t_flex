#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import time
from dynamixel_workbench_msgs.msg import DynamixelStateList
from t_flex.srv import JointSpeed, TorqueEnable
from t_flex.msg import GoalPosition
from std_msgs.msg import Bool, Float64
import os

class Controller(object):
    def __init__(self):
        home = os.path.expanduser('~')
        os.chdir(home + '/catkin_ws/src/t_flex/yaml')
        f = open("calibrationAngle.yaml", "r+")
        params = [f.readline().strip().split()[1] for i in range(4)]
        print(params)
        self.ValueToPubUp1 = float(params[0])
        self.ValueToPubDown1 = float(params[1])
        self.ValueToPubUp2 = float(params[2])
        self.ValueToPubDown2 = float(params[3])
        self.speed = 2*1024/10
        set_motor_speed(self.speed)
        rospy.init_node('therapy_with_flags', anonymous = True)
        self.command = rospy.Publisher("/goal_dynamixel_position", GoalPosition, queue_size = 1, latch = False)
        rospy.Subscriber("/kill_therapy", Bool, self.updateFlagTherapy)
        rospy.Subscriber("/update_speed", Float64, self.updateSpeed)
        rospy.Subscriber("/enable_device", Bool, self.updateFlagEnable)
        self.kill_therapy = False
        self.enable = None
        self.disabled = None

    def updateFlagTherapy(self, flag):
        self.kill_therapy = flag.data

    def updateSpeed(self, speed):
        self.speed = speed.data
        self.speed = self.speed*1024/10
        set_motor_speed(self.speed)

    def updateFlagEnable(self, flag):
        self.enable = flag.data
        if self.enable:
            rospy.loginfo("Assisting..")
        else:
            rospy.loginfo("Disabling Motors")

    def automatic_movement(self):
        rospy.loginfo("------------------------ THERAPY STARTED ------------------------")
        while not self.kill_therapy:
            if self.enable:
                ''' Position Publisher Motor ID 3 and Motor ID 4'''
                self.command.publish(id=[3,4],goal_position=[self.ValueToPubUp1,self.ValueToPubDown2])
                time.sleep(2)
                self.command.publish(id=[3,4],goal_position=[self.ValueToPubDown1,self.ValueToPubUp2])
                time.sleep(2)
                self.disabled = False
            else:
                if not self.disabled:
                    release_motors()
                    self.disabled = True
        rospy.loginfo("------------------------ THERAPY FINISHED -----------------------")

    def process(self):
        self.automatic_movement()
        release_motors()

def release_motors():
    val = False
    service = '/joint_torque_enable'
    rospy.wait_for_service(service)
    try:
         enable_torque = rospy.ServiceProxy(service, TorqueEnable)
         ''' Torque Disabled Motor ID 3 '''
         resp1 = enable_torque(id=3,torque_enable=val)
         time.sleep(0.001)
         ''' Torque Disabled Motor ID 4 '''
         resp2 = enable_torque(id=4,torque_enable=val)
         time.sleep(0.001)
         return (resp1.result & resp2.result)
    except rospy.ServiceException, e:
         print ("Service call failed: %s"%e)

def set_motor_speed(speed):
    val = speed
    service = '/joint_goal_speed'
    rospy.wait_for_service(service)
    try:
         motor_speed = rospy.ServiceProxy(service, JointSpeed)
         ''' Set Speed Motor ID 3 '''
         resp1 = motor_speed(id=3,set_speed=val)
         time.sleep(0.001)
         ''' Set Speed Motor ID 4 '''
         resp2 = motor_speed(id=4,set_speed=val)
         time.sleep(0.001)
         return (resp1.result & resp2.result)
    except rospy.ServiceException:
         print ("Service call failed: %s"%e)

def main():
    c = Controller()
    rospy.on_shutdown(release_motors)
    while not (rospy.is_shutdown()):
        c.process()
        break
    rospy.loginfo("Controller Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
