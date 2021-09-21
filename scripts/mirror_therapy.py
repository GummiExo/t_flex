#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time, os, sys, getopt, numpy, math
import rospy, rospkg, rosparam
from std_msgs.msg import Bool, Float64
from dynamixel_msgs.msg import MotorStateList
from dynamixel_controllers.srv import SetSpeed, TorqueEnable, SetKGain
from t_flex.msg import IMUData
import socket

class MirrorTherapyController(object):
    def __init__(self):
        ''' Inputs arguments for the node '''
        opts, args = getopt.getopt(sys.argv[1:], "t:", [])
        for opt, arg in opts:
            if opt == "-t" or "--time":
                self.time = int(arg)
        ''' Motors Parameters '''
        rospack = rospkg.RosPack()
        package_directory = rospack.get_path('t_flex')
        params = rosparam.load_file(package_directory + '/yaml/motors_parameters.yaml')
        self.max_value_motor1 = params[0][0]['tilt1_controller']['motor']['max']
        self.min_value_motor1 = params[0][0]['tilt1_controller']['motor']['min']
        self.init_value_motor1 = params[0][0]['tilt1_controller']['motor']['init']
        self.max_value_motor2 = params[0][0]['tilt2_controller']['motor']['max']
        self.min_value_motor2 = params[0][0]['tilt2_controller']['motor']['min']
        self.init_value_motor2 = params[0][0]['tilt2_controller']['motor']['init']
        ''' Node configuration '''
        rospy.init_node('t_flex_mirror_therapy', anonymous = True)
        ''' Variables '''
        self.isMotorAngleUpdated = False
        self.isPareticIMUUpated = False
        self.isNoPareticIMUUpated = False
        self.kill_mirror_therapy = False
        #self.euler_paretic = {'x': 0, 'y': 0, 'z': 0}
        #self.euler_no_paretic = {'x': 0, 'y': 0, 'z': 0}
        self.angle_paretic = 0
        self.angle_no_paretic = 0
        self.speed = 10 #Maximum velocity = 10
        self.calibration_time = 5 #seconds
        self.k_gain = 0.02
        self.initial_time = 0
        ''' Subscribers Initialization '''
        self.init_subscribers()
        ''' Publisher Initialization '''
        self.init_publishers()
        ''' Initial Configurations '''
        set_motor_speed(self.speed)
        time.sleep(0.1)
        pid_service(id=1,p=35,i=40,d=0) #Frontal Motor
        time.sleep(0.1)
        pid_service(id=2,p=36,i=36,d=0) #Posterior Motor
        ''' Socket Configuration '''
        self.mi_socket = socket.socket()
        self.mi_socket.connect(("192.168.4.8", 3014))

    def init_subscribers(self):
        rospy.Subscriber('/motor_states/tflex_tilt_port',MotorStateList, self.updateMotorsState)
        rospy.Subscriber('/imu_data/paretic', IMUData, self.updatePareticIMU)
        rospy.Subscriber('/imu_data/no_paretic', IMUData, self.updateNoPareticIMU)
        rospy.Subscriber('/kill_mirror_therapy', Bool, self.updateFlagTherapy)

    def init_publishers(self):
        self.frontal_motor_pub = rospy.Publisher('/tilt1_controller/command', Float64, queue_size = 1, latch = False)
        self.posterior_motor_pub = rospy.Publisher('/tilt2_controller/command', Float64, queue_size = 1, latch = False)
        self.kill_mirror_therapy_pub = rospy.Publisher('/kill_mirror_therapy', Bool, queue_size = 1, latch = False)
        self.kill_imu_acquisition_pub = rospy.Publisher('/kill_imu_acquisition', Bool, queue_size = 1, latch = False)

    def updateMotorsState(self, motor_info):
        try:
            self.MotorStateFrontal = motor_info.motor_states[0]
            self.MotorStatePosterior = motor_info.motor_states[1]
            self.isMotorAngleUpdated = True
        except:
            self.isMotorAngleUpdated = False

    def updatePareticIMU(self, imu_data):
        #self.euler_paretic = {'x': imu_data.euler_x, 'y': imu_data.euler_y, 'z': imu_data.euler_z}
        self.angle_paretic = imu_data.angle
        self.isPareticIMUUpated = True

    def updateNoPareticIMU(self, imu_data):
        #self.euler_no_paretic = {'x': imu_data.euler_x, 'y': imu_data.euler_y, 'z': imu_data.euler_z}
        self.angle_no_paretic = imu_data.angle
        self.isNoPareticIMUUpated = True

    def updateFlagTherapy(self, flag):
        self.kill_mirror_therapy = flag.data
        self.kill_imu_acquisition_pub.publish(True)
        rospy.loginfo("Killing nodes for mirror therapy")

    def position_to_radians(self,position,initial):
        rad = (position - initial)*2*math.pi/4095
        return rad

    def p_controller(self):
        angle_error = self.angle_no_paretic - self.angle_paretic
        #print("Angle error: " + str(angle_error))
        angle_threshold = 3 #Degrees
        if self.isPareticIMUUpated and self.isNoPareticIMUUpated and self.isMotorAngleUpdated:
            current_state_m1 = self.position_to_radians(self.MotorStateFrontal.position, self.init_value_motor1)
            current_state_m2 = self.position_to_radians(self.MotorStatePosterior.position, self.init_value_motor2)
            control_signal = self.k_gain*abs(angle_error)
            #print("Current state m1 and m2: " + str(current_state_m1) + "\t" + str(current_state_m2))
            if angle_error < -angle_threshold:
                if self.init_value_motor1 == self.min_value_motor1:
                    m1 = current_state_m1 + control_signal
                else:
                    m1 = current_state_m1 - control_signal
                if self.init_value_motor2 == self.min_value_motor2:
                    m2 = current_state_m2 - control_signal
                else:
                    m2 = current_state_m2 + control_signal
            elif angle_error > angle_threshold:
                if self.init_value_motor1 == self.min_value_motor1:
                    m1 = current_state_m1 - control_signal
                else:
                    m1 = current_state_m1 + control_signal
                if self.init_value_motor2 == self.min_value_motor2:
                    m2 = current_state_m2 + control_signal
                else:
                    m2 = current_state_m2 - control_signal
            else:
                return
            #m1 = 2*numpy.tanh(m1) # Saturation function for the error
            #m2 = 2*numpy.tanh(m2) # Saturation function for the error
            print("Value to pub m1 \t" + str(m1))
            print("Value to pub m2 \t" + str(m2))
            self.frontal_motor_pub.publish(m1)
            self.posterior_motor_pub.publish(m2)
            #TODO: Include socket command
            self.mi_socket.send(str(self.angle_no_paretic).encode("ascii"))
            self.isPareticIMUUpated = False
            self.isNoPareticIMUUpated = False
            self.isMotorAngleUpdated = False
            return
        else:
            return

    def time_control(self):
        current_time = time.time()
        if current_time - self.initial_time >= self.time:
            self.kill_mirror_therapy_pub.publish(True)
        return

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

def pid_service(id,p,i,d):
    ''' PID services '''
    p_service = 'tilt' + str(id) + '_controller/set_p_gain'
    i_service = 'tilt' + str(id) + '_controller/set_i_gain'
    d_service = 'tilt' + str(id) + '_controller/set_d_gain'
    ans = call_service(service=p_service, type=SetKGain, val = p)
    time.sleep(0.05)
    ans1 = call_service(service=i_service, type=SetKGain, val = i)
    time.sleep(0.05)
    ans2 = call_service(service=d_service, type=SetKGain, val = d)
    time.sleep(0.05)
    if ans and ans1 and ans2:
        rospy.loginfo("Successfull Gains Set (P: " + str(p) + " I: " + str(i) + " D: " + str(d))
    else:
        rospy.logerr("Gains set are not complete")

def call_service(service,type,val):
    rospy.wait_for_service(service)
    try:
        srv =  rospy.ServiceProxy(service, type)
        ans = srv(val)
        return ans
    except rospy.ServiceException, e:
         rospy.loginfo("Service call failed: %s"%e)

def main():
    c = MirrorTherapyController()
    rate = rospy.Rate(500)
    rospy.on_shutdown(release_motors)
    time.sleep(c.calibration_time + 2)
    c.initial_time = time.time()
    rospy.loginfo("Starting Mirror Therapy")
    while not (rospy.is_shutdown()):
        c.p_controller()
        #c.time_control()
        #if c.kill_mirror_therapy:
        #    break
        #rate.sleep()
    rospy.loginfo("Controller Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
