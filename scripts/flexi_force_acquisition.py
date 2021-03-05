#!/usr/bin/env python
import serial
import time
import os
import random
import rospy
import numpy
from std_msgs.msg import Float64,Int16
from scipy.signal import butter

class FlexiForceSensors(object):
    def __init__(self):
        #trial_name = "4cm_80_20"
        #trial_name = "4cm_70_30"
        trial_name = "4cm_60_40"
        #trial_name = "3cm_80_20"

        '''Parameters Inicialization '''
        self.serial_port = rospy.get_param("port","/dev/ttyACM0")
        os.system("sudo chmod 777 " + self.serial_port) #Enabling port permissions
        self.port_parameters = { "br": rospy.get_param("baud_rate", 9600) }
        self.ser = serial.Serial(self.serial_port, self.port_parameters["br"])
        self.pub_frontal = rospy.Publisher("frontal_flexiforce_data", Float64, queue_size = 1, latch = False)
        self.pub_posterior = rospy.Publisher("posterior_flexiforce_data", Float64, queue_size = 1, latch = False)
        self.pub_external = rospy.Publisher("external_flexiforce_data", Float64, queue_size = 1, latch = False)
        self.pub_internal = rospy.Publisher("internal_flexiforce_data", Float64, queue_size = 1, latch = False)
        self.pub_frontal_force = rospy.Publisher("frontal_flexiforce_force", Float64, queue_size = 1, latch = False)
        self.pub_posterior_force = rospy.Publisher("posterior_flexiforce_force", Float64, queue_size = 1, latch = False)
        self.pub_external_force = rospy.Publisher("external_flexiforce_force", Float64, queue_size = 1, latch = False)
        self.pub_internal_force = rospy.Publisher("internal_flexiforce_force", Float64, queue_size = 1, latch = False)
        ''' Node Configuration '''
        rospy.init_node('loadcell_sensor', anonymous = True)
        self.fs = 500
        ''' Flexiforce Configuration '''
        self.m_internal = 0.0257
        self.b_internal = 0.0748
        self.m_external = 0.0129
        self.b_external = -0.0902
        if trial_name == "4cm_80_20":
            self.m_posterior = 0.0149
            self.b_posterior = -0.0852
            self.m_frontal = 0.0088
            self.b_frontal = -0.0443
        elif trial_name == "4cm_70_30":
            self.m_posterior = 0.0193
            self.b_posterior = 0.0351
            self.m_frontal = 0.0060
            self.b_frontal = -0.0359
        elif trial_name == "4cm_60_40":
            self.m_posterior = 0.0164
            self.b_posterior = 0.0058
            self.m_frontal = 0.0557
            self.b_frontal = 0
        elif trial_name == "3cm_80_20":
            self.m_posterior = 0.0156
            self.b_posterior = 0.0214
            self.m_frontal = 0.3449
            self.b_frontal = -0.3964

        rospy.loginfo("Running Trial: " + trial_name)

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
        internal = 0
        frontal = 0
        posterior = 0
        external = 0
        try:
            data = self.ser.readline()
            #print((data))
            if len(data) == 26:
                internal = float(data[0:5])
                frontal = float(data[6:11])
                posterior = float(data[12:17])
                external = float(data[18:23])
        except:
            rospy.logwarn("Data not received")
        return (internal,frontal,posterior,external)

    def force(self,value,m,b):
        f = (value - b)/m
        return f


def main():
    flexiforce_sensors = FlexiForceSensors()
    rate = rospy.Rate(flexiforce_sensors.fs)
    #loadcell_sensor.initial_value()
    rospy.loginfo('Mining Data')
    while not rospy.is_shutdown():
        #flexiforce_sensors.read_data()
        internal,frontal,posterior,external = flexiforce_sensors.read_data()
        internal_force = flexiforce_sensors.force(internal,flexiforce_sensors.m_internal,flexiforce_sensors.b_internal);
        frontal_force = flexiforce_sensors.force(frontal,flexiforce_sensors.m_frontal,flexiforce_sensors.b_frontal);
        posterior_force = flexiforce_sensors.force(posterior,flexiforce_sensors.m_posterior,flexiforce_sensors.b_posterior);
        external_force = flexiforce_sensors.force(external,flexiforce_sensors.m_external,flexiforce_sensors.b_external);
        ''' Publishing Topics '''
        flexiforce_sensors.pub_frontal.publish(frontal)
        flexiforce_sensors.pub_posterior.publish(posterior)
        flexiforce_sensors.pub_external.publish(external)
        flexiforce_sensors.pub_internal.publish(internal)
        flexiforce_sensors.pub_frontal_force.publish(frontal_force)
        flexiforce_sensors.pub_posterior_force.publish(posterior_force)
        flexiforce_sensors.pub_external_force.publish(external_force)
        flexiforce_sensors.pub_internal_force.publish(internal_force)

        rate.sleep()
    flexiforce_sensors.ser.close()
    rospy.loginfo('Closed Port')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt as e:
        os.system('clear')
        print("Program finished\n")
        print e
        raise
