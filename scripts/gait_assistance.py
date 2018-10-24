#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys, getopt
import rospy
import time
from dynamixel_msgs.msg import MotorStateList
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64, Bool
from gummi_ankle.msg import GaitPhase
import threading
import logging
import os

logging.basicConfig(level = logging.DEBUG, format = '[%(levelname)s] (%(threadName)-9s) %(message)s',)

class Controller(object):
    def __init__(self):
        self.event = 0
        self.start_time = 0
        self.program_time = time.time()
        self.gait_phases = {1: "Heel strike",
                            2: "Flat foot",
                            3: "Midstance",
                            4: "Heel-off",
                            5: "Toe-off",
                            6: "Midswing",
                            7: "Invalid"}
        rospy.loginfo("------------------------- GAIT ASSISTANT ------------------------")
        rospy.loginfo("----------------------- ASSISTANT STARTED -----------------------")
        rospy.init_node('gait_assistance', anonymous = True)
        self.pub1 = rospy.Publisher("/tilt1_controller/command", Float64, queue_size = 1, latch = False)
        self.pub2 = rospy.Publisher("/tilt2_controller/command", Float64, queue_size = 1, latch = False)
        self.kill = rospy.Publisher("kill_gait_assistance", Bool, queue_size = 1, latch = False)
        self.flag = rospy.Subscriber("kill_gait_assistance", Bool, self.updateFlagGaitAssistance)
        rospy.Subscriber("/gait_phase_detection", GaitPhase, self.updateGaitEvent)
        opts, args = getopt.getopt(sys.argv[1:], "t:", [])
        for opt, arg in opts:
            if opt == "-t":
                self.time = int(arg)
        home = os.path.expanduser('~')
        os.chdir(home + '/catkin_ws/src/gummi_ankle/yaml')
        f = open("calibrationAngle.yaml", "r+")
        params = [f.readline().strip().split()[1] for i in range(4)]
        self.ValueToPubUp1 = float(params[0])
        self.ValueToPubDown1 = float(params[1])
        self.ValueToPubUp2 = float(params[2])
        # print(self.ValueToPubDown1)
        self.ValueToPubDown2 = float(params[3])
        os.chdir(home + '/catkin_ws/src/gummi_ankle/yaml')
        f = open("calibrationStiffness.yaml", "r+")
        params = [f.readline().strip().split()[1] for i in range(2)]
        self.StiffnessValueToPub1 = float(params[0])
        self.StiffnessValueToPub2 = float(params[1])
        send_request_motorSpeed1(1000)
        send_request_motorSpeed2(1000)
        self.program_time = time.time()

    def updateFlagGaitAssistance(self,kill_signal):
        if kill_signal.data:
            rospy.logwarn("Killing gait assistance node due to external source")
            rospy.signal_shutdown("Node was killed by external source.")

    def updateGaitEvent(self, sensor):
        """Is the given assistance time up?"""
        # print "Assistance time: " + str(time.time() - self.program_time) + " seconds"
        if time.time() - self.program_time > self.time:
            rospy.logwarn("Assistance time is up.")
            # rospy.logwarn("Releasing motors...")
            self.release_motors()
            kill_msg = Bool()
            kill_msg.data = True
            rospy.logwarn("Killing further nodes of gait assistance...")
            self.kill.publish(kill_msg)
            # self.gait_topic.unregister()
            rospy.signal_shutdown("Assistance time is up.")
        else:
            # Release motors if more than 1.5 seconds have passed between gait phases (No detection)
            if time.time() - self.start_time > 1.5:
                self.release_motors()
                self.start_time = time.time()
            if sensor.phase != self.event:
                self.start_time = time.time()
                self.event = sensor.phase
                rospy.loginfo("Gait Phase: "+self.gait_phases[self.event])
                ''' Heel Strike '''
                if self.event == 1:
                    time.sleep(0.005)
                    self.pub1.publish(self.StiffnessValueToPub1)
                    time.sleep(0.005)
                    self.pub2.publish(self.StiffnessValueToPub2)
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Heel Off '''
                if self.event == 4:
                    time.sleep(0.005)
                    self.pub1.publish(self.ValueToPubDown1)
                    time.sleep(0.005)
                    self.pub2.publish(self.ValueToPubUp2)
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Mid-Swing '''
                if self.event == 6:
                    time.sleep(0.005)
                    self.pub1.publish(self.ValueToPubUp1)
                    time.sleep(0.005)
                    self.pub2.publish(self.ValueToPubDown2)
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Invalide state! '''
                if self.event == 7:
                    print("Invalid state")
                    # Release the motors if an invalid state was prompted by the gait classifier
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
    # while not rospy.core.is_shutdown():
    #     rospy.rostime.wallsleep(0.5)
    # while not rospy.is_shutdown():
    rospy.spin()
        # pass
    rospy.on_shutdown(send_request_motor1)
    rospy.on_shutdown(send_request_motor2)
    rospy.loginfo("----------------------- ASSISTANT FINISHED ----------------------")

if __name__ == '__main__':
    main()
