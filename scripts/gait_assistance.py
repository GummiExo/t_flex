#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys, getopt
import rospy
import time
from dynamixel_workbench_msgs.msg import DynamixelStateList
from t_flex.srv import JointSpeed, TorqueEnable
from t_flex.msg import GoalPosition, GaitPhase
from std_msgs.msg import Bool
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
        self.command = rospy.Publisher("/goal_dynamixel_position", GoalPosition, queue_size = 1, latch = False)
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
        self.ValueToPubDown2 = float(params[3])
        os.chdir(home + '/catkin_ws/src/t_flex/yaml')
        f = open("calibrationStiffness.yaml", "r+")
        params = [f.readline().strip().split()[1] for i in range(2)]
        self.StiffnessValueToPub1 = float(params[0])
        self.StiffnessValueToPub2 = float(params[1])
        self.program_time = time.time()
        ''' Max Speed '''
        set_motor_speed(1024)

    def updateFlagGaitAssistance(self,kill_signal):
        if kill_signal.data:
            rospy.logwarn("Killing gait assistance node due to external source")
            release_motors()
            rospy.signal_shutdown("Node was killed by external source.")

    def updateGaitEvent(self, sensor):
        """Is the given assistance time up?"""
        # print "Assistance time: " + str(time.time() - self.program_time) + " seconds"
        if time.time() - self.program_time > self.time:
            rospy.logwarn("Assistance time is up.")
            # rospy.logwarn("Releasing motors...")
            release_motors()
            kill_msg = Bool()
            kill_msg.data = True
            rospy.logwarn("Killing further nodes of gait assistance...")
            self.kill.publish(kill_msg)
            rospy.signal_shutdown("Assistance time is up.")
        else:
            # Release motors if more than 1.5 seconds have passed between gait phases (No detection)
            if time.time() - self.start_time > 1.5:
                release_motors()
                self.start_time = time.time()
            if sensor.phase != self.event:
                self.start_time = time.time()
                self.event = sensor.phase
                rospy.loginfo("Gait Phase: "+self.gait_phases[self.event])
                ''' Heel Strike '''
                if self.event == 1:
                    self.command.publish(id=[3,4],goal_position=[self.StiffnessValueToPub1,self.StiffnessValueToPub2]
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Heel Off '''
                if self.event == 4:
                    self.command.publish(id=[3,4],goal_position=[self.ValueToPubDown1,self.ValueToPubUp2]
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Mid-Swing '''
                if self.event == 6:
                    self.command.publish(id=[3,4],goal_position=[self.ValueToPubUp1,self.ValueToPubDown2]
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Invalide state! '''
                if self.event == 7:
                    print("Invalid state")
                    # Release the motors if an invalid state was prompted by the gait classifier
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
    c = Controller()
    rospy.spin()
    rospy.loginfo("----------------------- ASSISTANT FINISHED ----------------------")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
