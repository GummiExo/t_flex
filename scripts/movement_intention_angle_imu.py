#!/usr/bin/python
"""
@author: Angie
"""
import rospy
from t_flex.msg import IMUData
from std_msgs.msg import Int16
import socket
import time
import scipy.io as sio
from scipy.signal import butter, lfilter
import numpy as np
import struct


class Calibracion(object):
    def __init__(self):
        self.datos = []
        self.ventana = []
        self.y = []
        self.promedios = []
        self.umbral = 0
        self.mi_socket = socket.socket()
        self.mi_socket.connect(("192.168.4.6", 3014))
        #ROS Node Initialization
        rospy.init_node('movement_intention_imu', anonymous = True)
        self.pub_paretic = rospy.Publisher('paretic_angle', Int16, queue_size = 1, latch = False)
        self.pub_nonparetic = rospy.Publisher('nonparetic_angle', Int16, queue_size = 1, latch = False)
        rospy.Subscriber('/imu_data_paretic', IMUData, self.updateIMUDataParetic)
        rospy.Subscriber('/imu_data_nonparetic', IMUData, self.updateIMUDataNonParetic)
        self.isIMUPareticUpdated = False
        self.isIMUNonPareticUpdated = False
        self.paretic_data = []
        self.nonparetic_data = []


    def updateIMUDataParetic(self,imu_data):
        self.isIMUPareticUpdated = True
        self.paretic_data = -imu_data.angle


    def updateIMUDataNonParetic(self,imu_data):
        self.isIMUNonPareticUpdated = True
        self.nonparetic_data = -imu_data.angle

    def butter_filtro_pasabajos(self, ventana):
        frec_corte = 6
        fs = 100
        orden = 4
        nyq = 0.5 * fs
        frecn = frec_corte / nyq
        b, a = butter(orden, frecn, btype='low', analog=False)
        y = lfilter(b, a, ventana)
        return y


    def juego_online(self):
        if ((self.isIMUPareticUpdated == True) and (self.isIMUNonPareticUpdated == True)):
            packer = struct.Struct('f')
            packed_data = packer.pack(self.nonparetic_data)
            self.mi_socket.sendall(packed_data)
            self.pub_paretic.publish(self.paretic_data)
            self.pub_nonparetic.publish(self.nonparetic_data)
            self.isIMUNonPareticUpdated = False
            self.isIMUPareticUpdated = False
        else:
            return

def main():
    nueva_sesion = Calibracion()
    rate = rospy.Rate(100)
    rospy.loginfo("Starting Angle Values for Paretic and Non-Paretic Joints")
    while not (rospy.is_shutdown()):
        nueva_sesion.juego_online()
        rate.sleep()


    rospy.loginfo("Controller Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
