#!/usr/bin/python
"""
@author: Angie
"""
import rospy
from t_flex.msg import IMUData
from std_msgs.msg import Bool
import socket
import time
import scipy.io as sio
from scipy.signal import butter, lfilter
import numpy as np


class Calibracion(object):
    def __init__(self):
        self.datos = []
        self.ventana = []
        self.y = []
        self.promedios = []
        self.umbral = 0
        self.mi_socket = socket.socket()
        self.mi_socket.connect(("192.168.4.9", 3014))
        #ROS Node Initialization
        rospy.init_node('movement_intention_imu', anonymous = True)
        self.pub = rospy.Publisher('movement_intention', Bool, queue_size = 1, latch = False)
        rospy.Subscriber('/imu_data', IMUData, self.updateIMUData)
        self.isIMUUpdated = False


    def updateIMUData(self,imu_data):
        if len(self.datos) >= 10:
            self.datos = np.delete(self.datos, 0)
            #self.datos = []
        self.isIMUUpdated = True
        self.datos = np.append(self.datos,imu_data.gyro_y)


    def butter_filtro_pasabajos(self, ventana):
        frec_corte = 6
        fs = 100
        orden = 4
        nyq = 0.5 * fs
        frecn = frec_corte / nyq
        b, a = butter(orden, frecn, btype='low', analog=False)
        y = lfilter(b, a, ventana)
        return y

    def deteccion_de_accion(self, y):
        j = 0
        for x in y:
            if x < 5.0:
                j = j + 1
            if j >= 9:
                accion = False
            else:
                accion = True
        return accion

    def revision_datos(self):
        if self.isIMUUpdated:
            if len(self.datos) >= 10:
                y = self.butter_filtro_pasabajos(self.datos)
                accion = self.deteccion_de_accion(y)
                #print("Hay accion: " + str(accion))
                if accion == True:
                    u = np.mean(y)
                    self.promedios.append(u)
                    self.mi_socket.send('jump'.encode())
                    time.sleep(0.5)
                    if len(self.promedios) == 5:
                        self.umbral = np.mean(self.promedios)
                        #print("Listo el umbral", self.umbral)
            else:
                time.sleep(0.01)
                return
            self.isIMUUpdated = False
        else:
            return

    def juego_online(self):
        if self.isIMUUpdated:
            y = self.butter_filtro_pasabajos(self.datos)
            u = np.mean(y)
            if u > self.umbral:
                #print(u, "es MAYOR")
                self.mi_socket.send('jump'.encode())
                self.pub.publish(True)
		time.sleep(0.01)
                #self.rta = self.mi_socket.recv(1024)
            else:
                self.pub.publish(False)
		time.sleep(0.01)
                #print(u, "es MENOR")
            self.isIMUUpdated = False
        else:
            return

def main():
    nueva_sesion = Calibracion()
    rate = rospy.Rate(200)
    rospy.loginfo("Starting Movement Intention Detector")
    while not (rospy.is_shutdown()):
        if len(nueva_sesion.promedios) <= 5:
            nueva_sesion.revision_datos()
            rospy.loginfo("Calibrating")
        else:
            nueva_sesion.juego_online()
        rate.sleep()


    rospy.loginfo("Controller Finished")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
