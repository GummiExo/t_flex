#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import time, os, sys
import rospy, rospkg
from std_msgs.msg import Bool, Float64
from dynamixel_msgs.msg import MotorStateList
from dynamixel_controllers.srv import SetSpeed, TorqueEnable
from t_flex.msg import IMUData


class MirrorTherapyController(object):
    def __init__(self):
        ''' Inputs arguments for the node '''
        opts, args = getopt.getopt(sys.argv[1:], "t:", [])
        for opt, arg in opts:
            if opt == "-t" || "--time":
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
        self.isNoPareticIMUUpated = True
        self.kill_therapy = False
        self.euler_paretic = {'x': 0, 'y': 0, 'z': 0}
        self.euler_no_paretic = {'x': 0, 'y': 0, 'z': 0}
        self.speed = 10 #Maximum velocity = 10
        self.calibration_time = 5 #seconds
        self.k_gain =
        self.initial_time = time.time()
        ''' Subscribers Initialization '''
        self.init_subscribers()
        ''' Publisher Initialization '''
        self.init_publishers()
        ''' Initial Configurations '''
        set_motor_speed(self.speed)
        self.initial_angle()

    def init_subscribers(self):
        rospy.Subscriber('/motor_states/tilt_port',MotorStateList, self.updateMotorsState)
        rospy.Subscriber('/paretic/imu_data', IMUData, self.updatePareticIMU)
        rospy.Subscriber('/no_paretic/imu_data', IMUData, self.updateNoPareticIMU)
        rospy.Subscriber('/kill_mirror_therapy', Bool, self.updateFlagTherapy)

    def init_publishers(self):
        self.frontal_motor_pub = rospy.Publisher('/tilt1_controller/command', Float64, queue_size = 1, latch = False)
        self.posterior_motor_pub = rospy.Publisher('/tilt2_controller/command', Float64, queue_size = 1, latch = False)
        self.kill_mirror_therapy_pub = rospy.Publisher('/kill_mirror_therapy', Bool, queue_size = 1, latch = False)

    def updateMotorsState(self, motor_info):
        try:
            self.MotorStateFrontal = motor_info.motor_states[0]
            self.MotorStatePosterior = motor_info.motor_states[1]
            self.position_to_radians()
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
        self.kill_therapy = flag.data

    def position_to_radians(self,flag):
        rad = (position - initial)*2*math.pi/4095
        return rad

    def p_controller(self):
        angle_error = self.angle_no_paretic - self.angle_paretic
        if self.isPareticIMUUpated and self.isNoPareticIMUUpated and self.isMotorAngleUpdated:}
            current_state_m1 = position_to_radians(self.MotorStateFrontal.position, self.init_value_motor1)
            current_state_m2 = position_to_radians(self.MotorStatePosterior.position, self.init_value_motor2)
            if angle_error > 0:
                control_signal = self._k_gain*angle_error
                if self.init_value_motor1 == self.min_value_motor1:
                    m1 = control_signal
                else
                    m1 = -control_signal
                if self.init_value_motor2 == self.min_value_motor2:
                    m2 = -control_signal
                else
                    m2 = control_signal
            self.frontal_motor_pub.publish(m1)
            self.posterior_motor_pub.publish(m2)


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
    c = TherapyController()
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
