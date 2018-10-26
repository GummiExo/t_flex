#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import time
from dynamixel_workbench_msgs.msg import DynamixelStateList
from t_flex.srv import JointSpeed, TorqueEnable
from t_flex.msg import GoalPosition
from std_msgs.msg import Bool
import os
import sys

class TherapyController(object):
    def __init__(self):
        self.repeats, self.frecuency, self.speed = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
        ''' Max Speed Value: 1024 '''
        self.speed = self.speed*1024/10
        home = os.path.expanduser('~')
        os.chdir(home + '/catkin_ws/src/t_flex/yaml')
        f = open("calibrationAngle.yaml", "r+")
        params = [f.readline().strip().split()[1] for i in range(4)]
        print(params)
        rospy.init_node('therapy', anonymous = True)
        self.ValueToPubUp1 = float(params[0])
        self.ValueToPubDown1 = float(params[1])
        self.ValueToPubUp2 = float(params[2])
        self.ValueToPubDown2 = float(params[3])
        set_motor_speed(self.speed)
        self.command = rospy.Publisher("/goal_dynamixel_position", GoalPosition, queue_size = 1, latch = False)
        rospy.Subscriber("/kill_therapy", Bool, self.updateFlagTherapy)
        self.kill_therapy = False

    def updateFlagTherapy(self, flag):
        self.kill_therapy = flag.data

    def automatic_movement(self):
        rospy.loginfo("------------------------ THERAPY STARTED ------------------------")
        for n in range (0,self.repeats):
            if not self.kill_therapy:
                ''' Position Publisher Motor ID 3 and Motor ID 4'''
                self.command.publish(id=[3,4],goal_position=[self.ValueToPubUp1,self.ValueToPubDown2])
                time.sleep(1/self.frecuency)
                self.command.publish(id=[3,4],goal_position=[self.ValueToPubDown1,self.ValueToPubUp2])
                time.sleep(1/self.frecuency)
            else:
                break
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
         print "Service call failed: %s"%e

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
    c = TherapyController()
    while not (rospy.is_shutdown()):
        c.process()
        break
    rospy.loginfo("Controller Finished")
    rospy.on_shutdown(release_motors)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
