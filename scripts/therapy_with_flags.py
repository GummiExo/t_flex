#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import time
from dynamixel_controllers.srv import SetSpeed, TorqueEnable
from std_msgs.msg import Bool, Float64
import os
import sys

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
        self.frecuency = 0.5
        self.speed = 1
        set_motor_speed(self.speed)
        rospy.init_node('t_flex_therapy_with_flags', anonymous = True)
        rospy.Subscriber("/t_flex/kill_therapy", Bool, self.updateFlagTherapy)
        rospy.Subscriber("/t_flex/update_speed", Float64, self.updateSpeed)
        rospy.Subscriber("enable_device", Bool, self.updateFlagEnable)
        self.frontal_motor_pub = rospy.Publisher("/tilt1_controller/command", Float64, queue_size = 1, latch = False)
        self.posterior_motor_pub = rospy.Publisher("/tilt2_controller/command", Float64, queue_size = 1, latch = False)
        self.kill_therapy = False
        self.enable = None
        self.disabled = None

    def updateFlagTherapy(self, flag):
        self.kill_therapy = flag.data

    def updateSpeed(self, speed):
        self.speed = speed.data
        set_motor_speed(self.speed)

    def updateFlagEnable(self, flag):
        self.enable = flag.data
        if self.enable:
            rospy.loginfo("Assisting Dorsiflexion")
        else:
            rospy.loginfo("Assisting Platarflexion")

    def automatic_movement(self):
        #rospy.loginfo("------------------------ THERAPY STARTED ------------------------")
        if self.enable:
            ''' Publisher Motor ID 1 and Motor ID 2'''
            self.frontal_motor_pub.publish(self.ValueToPubUp1)
            self.posterior_motor_pub.publish(self.ValueToPubDown2)
                #time.sleep(1/self.frecuency)
                #self.frontal_motor_pub.publish(self.ValueToPubDown1)
                #self.posterior_motor_pub.publish(self.ValueToPubUp2)
                #time.sleep(1/self.frecuency)
                #self.disabled = False
        else:
            self.frontal_motor_pub.publish(self.ValueToPubDown1)
            self.posterior_motor_pub.publish(self.ValueToPubUp2)
                
                    #release_motors()
                    #self.disabled = True
        time.sleep(0.5)
        #rospy.loginfo("------------------------ THERAPY FINISHED -----------------------")

    def process(self):
        self.automatic_movement()
        #release_motors()

def release_motors():
    value = False
    type_service = TorqueEnable
    service_frontal = '/tilt1_controller/torque_enable'
    service_posterior = '/tilt2_controller/torque_enable'
    ans = call_service(service=service_frontal, type=type_service, val = value)
    ans = call_service(service=service_posterior, type=type_service, val = value)

def set_motor_speed(speed):
    value = speed
    type_service = SetSpeed
    service_frontal = '/tilt1_controller/set_speed'
    service_posterior = '/tilt2_controller/set_speed'
    ans = call_service(service=service_frontal, type=type_service, val = value)
    ans = call_service(service=service_posterior, type=type_service, val = value)

def call_service(service,type,val):
    rospy.wait_for_service(service)
    try:
        srv =  rospy.ServiceProxy(service, type)
        ans = srv(val)
        return ans
    except rospy.ServiceException, e:
         rospy.loginfo("Service call failed: %s"%e)

def main():
    c = Controller()
    #rospy.on_shutdown(release_motors)
    while not (rospy.is_shutdown()):
        c.automatic_movement()
        if c.kill_therapy:
            break
    rospy.loginfo("Controller Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
