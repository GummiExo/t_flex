#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
import os
import sys
import numpy as np
import RPi.GPIO as GPIO
from gummi_ankle.msg import Insole


class InsoleSensors(object):
    def __init__(self, toe=4, l_meta=17, r_meta=27, heel=22):
        self.toe_pin = toe
        self.l_meta_pin = l_meta
        self.r_meta_pin = r_meta
        self.heel_pin = heel
        # self.start_time = time.time()
        self.node_name = "fsr_data_acquisition"
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.toe_pin, GPIO.IN)
        GPIO.setup(self.l_meta_pin, GPIO.IN)
        GPIO.setup(self.r_meta_pin, GPIO.IN)
        GPIO.setup(self.heel_pin, GPIO.IN)
        rospy.init_node(self.node_name, anonymous = True)
        rospy.loginfo("INPUTS:\nPIN\tSENSOR\n%i\tTOE\n%i\tLEFT METATARSAL\n%i\tRIGHT METATARSAL\n%i\tHEEL",self.toe_pin,self.l_meta_pin,self.r_meta_pin,self.heel_pin)
        self.pub = rospy.Publisher("/fsr_data", Insole, queue_size = 1, latch = False)

    def read_data(self):
        self.toe = GPIO.input(self.toe_pin)
        self.r_meta = GPIO.input(self.r_meta_pin)
        self.l_meta = GPIO.input(self.l_meta_pin)
        self.heel = GPIO.input(self.heel_pin)

def main():
    s = InsoleSensors(toe=4, l_meta=17, r_meta=27, heel=22)
    msg = Insole()
    rate = rospy.Rate(650)
    print('Reading FSR data, press Ctrl-C to quit...')
    while not rospy.is_shutdown():
        s.read_data()
        msg.header.frame_id = "/" + s.node_name
        # msg.time_stamp = int(round((time.time() - s.start_time)*1000.0))
        msg.toe = s.toe
        msg.r_meta = s.r_meta
        msg.l_meta = s.l_meta
        msg.heel = s.heel
        s.pub.publish(msg)
        # time.sleep(0.5)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.loginfo("Program Finished\n")
        print e
        sys.stdout.close()
        raise
