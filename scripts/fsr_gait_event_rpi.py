#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
from gummi_ankle.msg import GaitEvent
import RPi.GPIO as GPIO

class InsoleSensors(object):
    def __init__(self):
        self.toe = 4
        self.l_meta = 17
        self.r_meta = 27
        self.heel = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.toe, GPIO.IN)
        GPIO.setup(self.l_meta, GPIO.IN)
        GPIO.setup(self.r_meta, GPIO.IN)
        GPIO.setup(self.heel, GPIO.IN)
        rospy.init_node('gait_event_from_fsr', anonymous = True)
        rospy.loginfo("INPUTS:\nPIN\tSENSOR\n%i\tTOE\n%i\tLEFT METATARSAL\n%i\tRIGHT METATARSAL\n%i\tHEEL",self.toe,self.l_meta,self.r_meta,self.heel)
        self.pub = rospy.Publisher('/fsr_gait_event', GaitEvent, queue_size = 1, latch = False)
        self.event = []

    def process(self):
        msg = GaitEvent()
        ''' Gait Events:
            1 STANCE PHASE
            2 TOE OFF
            3 SWING PHASE
            4 HEEL STRIKE
        '''
        if GPIO.input(self.heel) == 1 and (GPIO.input(self.l_meta) or GPIO.input(self.r_meta)) == 1:
            self.event = np.append(self.event,1)
            print "STANCE PHASE"
        if GPIO.input(self.heel) == 0 and GPIO.input(self.toe) == 1 and (GPIO.input(self.l_meta) or GPIO.input(self.r_meta)) == 1:
            self.event = np.append(self.event,2)
            print "TOE OFF"
        if GPIO.input(self.toe) == 0 and GPIO.input(self.l_meta) == 0 and GPIO.input(self.r_meta) == 0 and GPIO.input(self.toe) == 0:
            self.event = np.append(self.event,3)
            print "SWING PHASE"
        if GPIO.input(self.heel) == 1 and GPIO.input(self.toe) == 0:
            self.event = np.append(self.event,4)
            print "HEEL STRIKE"
        if self.event[len(self.event)-1] != self.event[len(self.event)-2]:
            msg.event = self.event[len(self.event)-1]
 	    self.pub.publish(msg)
	    print msg.event
        time.sleep(0.01)

def main():
    s = InsoleSensors()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        s.process()
        rate.sleep()
    filename = 'fsr_gait_event.txt'
    with open (filename,'w+') as f
        f.write(s.event)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.loginfo("Program Finished")
