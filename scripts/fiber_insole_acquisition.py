#!/usr/bin/env python
import serial
import time
import os
import random
import rospy
import numpy
import glob
from scipy.signal import butter, filtfilt, medfilt, detrend, firwin, lfilter
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray, Pose

class FiberInsole(object):
    def __init__(self):

        '''Parameters Inicialization '''
        self.serial_port = rospy.get_param("port","/dev/rfcomm6")
        os.system("sudo chmod 777 " + self.serial_port) #Enabling port permissions
        self.port_parameters = { "br": rospy.get_param("baud_rate", 115200) }
        self.ser = serial.Serial(port=self.serial_port, baudrate=self.port_parameters["br"],parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        self.pub_insole_adc = rospy.Publisher("fiber_insole", PoseArray, queue_size = 1, latch = False)
        self.i = 0
        self.toe_vector = [0]
        self.ball_vector =[0]
        self.arch_vector =[0]
        self.heel_vector =[0]
        self.toe_filt = 0
        self.ball_filt = 0
        self.arch_filt = 0
        self.heel_filt = 0
        self.toe_detrend = 0
        self.ball_detrend = 0
        self.arch_detrend = 0
        self.heel_detrend = 0
        ''' Node Configuration '''
        rospy.init_node('fiber_insole', anonymous = True)
        self.fs = 500
        ''' Filter Configuration '''
        self.fs_filter = 200
        self.cutoff = 1 # Hz
        self.nyq = 0.5*self.fs_filter
        self.order = 2
        self.b_filter, self.a_filter = self.butter_lowpass_filter(self.cutoff,self.fs_filter,self.order,self.nyq)
        self.n = 10
        self.fc = 0.001
        self.h = self.trend_filter(self.n, self.fc, self.fs)
        #self.m_internal = 0.0257
        #self.b_internal = 0.0748
        #self.m_external = 0.0129
        #self.b_external = -0.0902

        #rospy.loginfo("Running Trial: " + trial_name)
    def trend_filter(self,n,fc,fs):
        h = firwin(n,fc,fs/2)
        h = -h
        return h


    def butter_lowpass_filter(self,cutoff,fs,order,nyq):
        normal_cutoff = cutoff/nyq
        b,a = butter(order,normal_cutoff, btype='low',analog=False)
        return b,a


    def initial_value(self):
        ''' Calculating Initial Values '''
        n = 0
        mean_value_posterior = []
        mean_value_frontal = []
        while n<=200:
            value_posterior,value_frontal = self.read_data();
            if value_posterior != 0.0:
                mean_value_posterior = numpy.append(mean_value_posterior, value_posterior)
            if value_frontal != 0.0:
                mean_value_frontal = numpy.append(mean_value_frontal, value_frontal)
            n += 1
        print("Frontal")
        print(mean_value_frontal)
        print("Posterior")
        print(mean_value_posterior)
        self.b1 = numpy.mean(mean_value_posterior[0:10], axis=0, dtype=numpy.float64)
        self.b2 = numpy.mean(mean_value_frontal[-50::], axis=0, dtype=numpy.float64)
        rospy.loginfo("Initial Value Frontal: %s",self.b2)
        rospy.loginfo("Initial Value Posterior: %s",self.b1)

    def read_data(self):
        toe = 0
        ball = 0
        arch = 0
        heel = 0
        data = self.ser.readline()
        try:
            data = self.ser.readline()
            #print((data[18:23]))
            #print((data))
            if len(data) == 24:
                toe = float(data[0:5])
                ball = float(data[6:11])
                arch = float(data[12:17])
                heel = float(data[18:23])
                if len(self.toe_vector) < self.n:
                    self.toe_vector.append(toe)
                    self.ball_vector.append(ball)
                    self.arch_vector.append(arch)
                    self.heel_vector.append(heel)
                else:
                    #self.toe_filt = int(filtfilt(self.b_filter,self.a_filter,self.toe_vector).tolist().pop())
                    #self.ball_filt = int(filtfilt(self.b_filter,self.a_filter,self.ball_vector).tolist().pop())
                    #self.arch_filt = int(filtfilt(self.b_filter,self.a_filter,self.arch_vector).tolist().pop())
                    #self.heel_filt = int(filtfilt(self.b_filter,self.a_filter,self.heel_vector).tolist().pop())
                    ''' Medfilt '''
                    toe_filt_vector = (medfilt(self.toe_vector))
                    ball_filt_vector = (medfilt(self.ball_vector))
                    arch_filt_vector = (medfilt(self.arch_vector))
                    heel_filt_vector = (medfilt(self.heel_vector))
                    self.toe_filt = int(toe_filt_vector.tolist().pop())
                    self.ball_filt = int(ball_filt_vector.tolist().pop())
                    self.arch_filt = int(arch_filt_vector.tolist().pop())
                    self.heel_filt = int(heel_filt_vector.tolist().pop())
                    ''' Delete Data Trend '''
                    #self.toe_detrend = int(detrend(toe_filt_vector).tolist().pop())
                    self.toe_detrend = lfilter(self.h,100,toe_filt_vector)
                    print(self.toe_detrend)
                    # self.ball_detrend = int(detrend(ball_filt_vector).tolist().pop())
                    # self.arch_detrend = int(detrend(arch_filt_vector).tolist().pop())
                    # self.heel_detrend = int(detrend(heel_filt_vector).tolist().pop())
                    self.toe_vector.pop(0)
                    self.ball_vector.pop(0)
                    self.arch_vector.pop(0)
                    self.heel_vector.pop(0)
                    # self.toe_detrend.pop(0)
                    # self.ball_detrend.pop(0)
                    # self.arch_detrend.pop(0)
                    # self.heel_detrend.pop(0)
                #print(float(toe))
                return (toe,ball,arch,heel,self.toe_filt,self.ball_filt,self.arch_filt,self.heel_filt,self.toe_detrend,self.ball_detrend,self.arch_detrend,self.heel_detrend)
            else:
                rospy.logwarn("Incomplete Data")
                #print((data))
                #self.i = self.i+1
                #print(self.i)
                return (-1,-1,-1,-1,0,0,0,0,0,0,0,0)
        except:
            rospy.logwarn("Data not received")
            return (-1,-1,-1,-1,0,0,0,0,0,0,0,0)

    def force(self,value,m,b):
        f = (value - b)/m
        return f


def main():
    fiber_insole = FiberInsole()
    rate = rospy.Rate(fiber_insole.fs)
    #loadcell_sensor.initial_value()
    rospy.loginfo('Mining Data')
    while not rospy.is_shutdown():
        toe_adc,ball_adc,arch_adc,heel_adc,toe_adc_filt,ball_adc_filt,arch_adc_filt,heel_adc_filt,toe_adc_detrend,ball_adc_detrend,arch_adc_detrend,heel_adc_detrend  = fiber_insole.read_data()
        if not ((toe_adc == -1) and (ball_adc == -1) and (arch_adc == -1) and (heel_adc == -1)):

            adc_msg = PoseArray()
            adc_vector_msg = [Pose(),Pose(),Pose(),Pose()]
            adc_vector_msg[0].position.x = toe_adc
            adc_vector_msg[1].position.x = ball_adc
            adc_vector_msg[2].position.x = arch_adc
            adc_vector_msg[3].position.x = heel_adc
            adc_vector_msg[0].position.y = toe_adc_filt
            adc_vector_msg[1].position.y = ball_adc_filt
            adc_vector_msg[2].position.y = arch_adc_filt
            adc_vector_msg[3].position.y = heel_adc_filt
            adc_vector_msg[0].position.z = toe_adc_detrend
            adc_vector_msg[1].position.z = ball_adc_detrend
            adc_vector_msg[2].position.z = arch_adc_detrend
            adc_vector_msg[3].position.z = heel_adc_detrend
            #print((voltage_vector_msg))

            adc_msg.header.stamp = rospy.Time.now()
            adc_msg.poses = adc_vector_msg
            fiber_insole.pub_insole_adc.publish(adc_msg)
        rate.sleep()
    fiber_insole.ser.close()
    rospy.loginfo('Closed Port')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt as e:
        os.system('clear')
        print("Program finished\n")
        print e
        raise
