#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import time
from dynamixel_msgs.msg import MotorStateList
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64, Bool
from gummi_ankle.msg import GaitEvent
import os

class Controller(object):
    def __init__(self):
        home = os.path.expanduser('~')
        os.chdir(home + '/catkin_ws/src/gummi_ankle/yaml')
        f = open("calibrationAngle.yaml", "r+")
        params = [f.readline().strip().split()[1] for i in range(4)]
        print(params)
        self.ValueToPubUp1 = float(params[0])
        self.ValueToPubDown1 = float(params[1])
        self.ValueToPubUp2 = float(params[2])
        self.ValueToPubDown2 = float(params[3])
        self.speed = 3
        send_request_motorSpeed1(self.speed)
        time.sleep(0.1)
        send_request_motorSpeed2(self.speed)
        rospy.init_node('auto_movement_with_flags', anonymous = True)
        self.pub1 = rospy.Publisher("/tilt1_controller/command", Float64, queue_size = 1, latch = False)
        self.pub2 = rospy.Publisher("/tilt2_controller/command", Float64, queue_size = 1, latch = False)
        rospy.Subscriber("/flag_therapy", Bool, self.updateFlagTherapy)
        rospy.Subscriber("/update_speed", Float64, self.updateSpeed)
        rospy.Subscriber("/enable_device", Bool, self.updateFlagEnable)
        self.running = True
        self.enable = None

    def updateFlagTherapy(self, data):
        self.running = data.data

    def updateSpeed(self, data):
        self.speed = data.data
        send_request_motorSpeed1(self.speed)
        time.sleep(0.1)
        send_request_motorSpeed2(self.speed)

    def self.updateFlagEnable(self, data):
        self.enable = data.data

    def automatic_movement(self):
        rospy.loginfo("------------------------ THERAPY STARTED ------------------------")
        while self.running:
            if self.enable:
                self.pub1.publish(self.ValueToPubUp1)
                self.pub2.publish(self.ValueToPubDown2)
                time.sleep(1)
                self.pub1.publish(self.ValueToPubDown1)
                self.pub2.publish(self.ValueToPubUp2)
                time.sleep(1)
            else:
                self.release_motors()
                self.enable = None
        rospy.loginfo("------------------------ THERAPY FINISHED -----------------------")

    def process(self):
        self.automatic_movement()
        self.release_motors()

    def release_motors(self):
        send_request_motor1()
        send_request_motor2()

def send_request_motor1():
    val = False
    service = '/tilt1_controller/torque_enable'
    rospy.wait_for_service(service)
    try:
         enable_torque = rospy.ServiceProxy(service, TorqueEnable)
         resp = enable_torque(val)
         rospy.loginfo("Torque Disabled Motor 1")
         return resp
    except rospy.ServiceException, e:
         print "Service call failed: %s"%e

def send_request_motor2():
    val = False
    service = '/tilt2_controller/torque_enable'
    rospy.wait_for_service(service)
    try:
         enable_torque = rospy.ServiceProxy(service, TorqueEnable)
         resp = enable_torque(val)
         rospy.loginfo("Torque Disabled Motor 2")
         return resp
    except rospy.ServiceException, e:
         print "Service call failed: %s"%e

def send_request_motorSpeed1(speed):
    val = speed
    service = '/tilt1_controller/set_speed'
    rospy.wait_for_service(service)
    try:
         motor_speed = rospy.ServiceProxy(service, SetSpeed)
         resp = motor_speed(val)
         return resp
    except rospy.ServiceException:
         print ("Service call failed: %s"%e)

def send_request_motorSpeed2(speed):
    val = speed
    service = '/tilt2_controller/set_speed'
    rospy.wait_for_service(service)
    try:
         motor_speed = rospy.ServiceProxy(service, SetSpeed)
         resp = motor_speed(val)
         return resp
    except rospy.ServiceException:
         print ("Service call failed: %s"%e)

def main():
    c = Controller()
    while not (rospy.is_shutdown()):
        c.process()
        break
    rospy.loginfo("Controller Finished")
    rospy.on_shutdown(send_request_motor1)
    rospy.on_shutdown(send_request_motor2)

if __name__ == '__main__':
    main()
