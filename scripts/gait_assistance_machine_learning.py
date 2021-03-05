#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys, getopt
import rospy, rospkg
import time
from dynamixel_controllers.srv import SetSpeed, TorqueEnable
from std_msgs.msg import Bool, Float64, Int8
import threading
import logging
import os

logging.basicConfig(level = logging.DEBUG, format = '[%(levelname)s] (%(threadName)-9s) %(message)s',)

class Controller(object):
    def __init__(self):
        self.event = 0
        self.start_time = 0
        self.program_time = time.time()
        self.gait_phases = {0: "Heel strike",
                            1: "Flat foot",
                            2: "Heel-off",
                            3: "Toe-off"}
        rospy.loginfo("------------------------- GAIT ASSISTANT ------------------------")
        rospy.loginfo("----------------------- ASSISTANT STARTED -----------------------")
        rospy.init_node('t_flex_gait_assistance', anonymous = True)
        # self.command = rospy.Publisher("/goal_dynamixel_position", GoalPosition, queue_size = 1, latch = False)
        self.kill = rospy.Publisher("/kill_gait_assistance", Bool, queue_size = 1, latch = False)
        self.flag = rospy.Subscriber("/kill_gait_assistance", Bool, self.updateFlagGaitAssistance)
        self.frontal_motor_pub = rospy.Publisher("/tilt1_controller/command", Float64, queue_size = 1, latch = False)
        self.posterior_motor_pub = rospy.Publisher("/tilt2_controller/command", Float64, queue_size = 1, latch = False)
        rospy.Subscriber("/gait_phase", Int8, self.updateGaitEvent)
        opts, args = getopt.getopt(sys.argv[1:], "t:", [])
        for opt, arg in opts:
            if opt == "-t":
                self.time = int(arg)
        rospack = rospkg.RosPack()
        package_directory = rospack.get_path('t_flex')
        os.chdir(package_directory + '/yaml')
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
        set_motor_speed(10)

    def updateFlagGaitAssistance(self,kill_signal):
        if kill_signal.data:
            rospy.logwarn("Killing gait assistance node due to external source")
            release_motors()
            #rospy.signal_shutdown("Node was killed by external source.")

    def flexion_movement(self):
        self.frontal_motor_pub.publish(self.ValueToPubUp1)
        self.posterior_motor_pub.publish(self.ValueToPubDown2)

    def extension_movement(self):
        self.frontal_motor_pub.publish(self.ValueToPubDown1)
        self.posterior_motor_pub.publish(self.ValueToPubUp2)

    def maximum_stiffness(self):
        self.frontal_motor_pub.publish(self.StiffnessValueToPub1)
        self.posterior_motor_pub.publish(self.StiffnessValueToPub2)

    def updateGaitEvent(self, sensor):
        """Is the given assistance time up?"""
        # print "Assistance time: " + str(time.time() - self.program_time) + " seconds"
        if time.time() - self.program_time > self.time:
            rospy.logwarn("Assistance time is up.")
            # rospy.logwarn("Releasing motors...")
            #release_motors()
            kill_msg = Bool()
            kill_msg.data = True
            rospy.logwarn("Killing further nodes of gait assistance...")
            #self.kill.publish(kill_msg)
            rospy.signal_shutdown("Assistance time is up.")
        else:
            # Release motors if more than 1.5 seconds have passed between gait phases (No detection)
            if time.time() - self.start_time > 1.5:
                #release_motors()
                self.start_time = time.time()
            if sensor.data != self.event:
                self.start_time = time.time()
                self.event = sensor.data
                rospy.loginfo("Gait Phase: "+self.gait_phases[self.event])
                ''' Heel Strike '''
                if self.event == 1:
		    #self.extension_movement()
                    self.maximum_stiffness()
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Heel Off '''
                if self.event == 2:
                    self.extension_movement()
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Toe-Off '''
                if self.event == 3:
                    self.flexion_movement()
                    print("\nPublishing: " + self.gait_phases[self.event] + "\n")
                ''' Invalide state! '''
                if self.event == 7:
                    rospy.logerrn("Invalid state")
                    # Release the motors if an invalid state was prompted by the gait classifier
                    release_motors()

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
    rospy.on_shutdown(release_motors)
    rospy.spin()
    rospy.loginfo("----------------------- ASSISTANT FINISHED ----------------------")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
