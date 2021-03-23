#!/usr/bin/python
import rospy
import time
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from t_flex.msg import IMUData
import numpy as np
import scipy.io as sio
from scipy.signal import butter, lfilter


class MovementIntention(object):
    def __init__(self):
        ''' EMG '''
        self.ventana = []
        self.umbral = 0
        self.muestras = 500
        self.estado = 'sin'
        ''' IMU '''
        self.datos = []
        self.y = []
        self.promedios = []
        
        #self.operador = "MEAN"
        
        ''' ROS commands '''
        rospy.init_node('movement_intention_detector', anonymous = True)
        self.initParameters()
        self.initSubscribers()
        self.pub_intention = rospy.Publisher("/movement_intention", Header, queue_size = 1, latch = False)
        self.isIMUUpdated = False
        self.isEMGUpdated = False  
        
    def initParameters(self):
        self.selected_sensor = rospy.get_param("~sensor","emg") #emg, imu
        self.selected_method = rospy.get_param("~method","mean") #mean, std, var, mean_std, mav
        return
    
    def initSubscribers(self):
        if self.selected_sensor == 'emg':
            rospy.Subscriber("/emg_data", PointStamped, self.updateEMGData)
        elif self.selected_sensor == 'imu':
            rospy.Subscriber('/imu_data', IMUData, self.updateIMUData)
        

    def updateIMUData(self,imu_data):
        if len(self.datos) >= 31:
            self.datos = np.delete(self.datos, 0)
            #self.datos = []
        self.isIMUUpdated = True
        self.datos = np.append(self.datos,imu_data.gyro_y)
        
    def updateEMGData(self,emg_data):
        if len(self.ventana) >= self.muestras:
            self.ventana = np.delete(self.ventana, 0)
        self.isEMGUpdated = True
        self.ventana = np.append(self.ventana,emg_data.point.x)
        
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

    def calculo_umbral_imu(self):
        if self.isIMUUpdated:
            if len(self.datos) >= 31:
                y = self.butter_filtro_pasabajos(self.datos)
                accion = self.deteccion_de_accion(y)
                #print("Hay accion: " + str(accion))
                if accion == True:
                    u = np.mean(y)
                    self.promedios.append(u)
                    if len(self.promedios) == 5:
                        if self.selected_method == 'mean':
                            self.umbral = np.mean(self.promedios)
                        elif self.selected_method == 'std':
                            self.umbral = np.std(self.promedios)
                        elif self.selected_method == 'var':
                            self.umbral = np.var(self.promedios)
                        elif self.selected_method == 'mean_std':
                            self.umbral = np.mean(self.promedios) + (3*np.std(self.promedios))
                        elif self.selected_method == 'mav':
                            self.umbral = np.mean(np.abs(np.diff(self.promedios)))
                        else:
                            rospy.logerr("Method not selected")
                        #print("Listo el umbral", self.umbral)
            else:
                return
            self.isIMUUpdated = False
        else:
            return
        
    def calculo_umbral_emg(self):
        if self.isEMGUpdated:
            if ((len(self.ventana) == self.muestras) and (self.estado == 'sin')):
                if self.selected_method == 'mean':
                    self.umbral = np.mean(self.ventana)
                    self.estado = 'con'
                    print(self.estado)
                elif self.selected_method == 'std':
                    self.umbral = np.std(self.ventana)
                    self.estado = 'con'
                elif self.selected_method == 'var':
                    self.umbral = np.var(self.ventana)
                    self.estado = 'con'
                elif self.selected_method == 'mean_std':
                    self.umbral = np.mean(self.ventana) + (3*np.std(self.ventana))
                    self.estado = 'con'
                elif self.selected_method == 'mav':
                    self.umbral = np.mean(np.abs(np.diff(self.ventana)))
                    self.estado = 'con'
                else:
                    rospy.logerr("Method not selected")
                return
            else:
                return
            self.isEMGUpdated = False
        else:
            return
        
    
    def operacion_ventana(self,ventana,tipo):
        if tipo == 'mean':
            valor = np.mean(ventana)
        if tipo == 'std':
            valor = np.std(ventana)
        if tipo == 'var':
            valor = np.var(ventana)
        if tipo == "mean_std":
            valor = np.mean(ventana) + (3*np.std(ventana))
        if tipo == "mav":
            valor = np.mean(np.abs(np.diff(ventana)))
        return valor
        

def main():
    p = MovementIntention()
    rate = rospy.Rate(2000)
    rospy.loginfo("Starting Movement Intention Detector %s",p.selected_sensor)
    msg = Header()
    while not (rospy.is_shutdown()):
        if ((p.selected_sensor == 'imu') or (p.selected_sensor == 'IMU')):
            if p.isIMUUpdated:
                if len(p.promedios) <= 5:
                    p.calculo_umbral_imu()
                else:
                    #rospy.loginfo("IMU Threshold Calculated: %s",p.umbral)
                    ahora = p.operacion_ventana(p.datos[-30],p.selected_method)
                    if ahora > p.umbral:
                        msg.frame_id = "imu_intention_" + p.selected_method
                        msg.stamp = rospy.Time.now()
                        msg.seq = ahora
                        p.pub_intention.publish(msg)
                        time.sleep(0.1)
            else:
                pass
        elif ((p.selected_sensor == 'emg') or (p.selected_sensor == 'EMG')):
            if p.isEMGUpdated:
                if p.estado == 'sin':
                    p.calculo_umbral_emg()
                else:
                    #rospy.loginfo("EMG Threshold Calculated: %s", p.umbral)
                    ahora = p.operacion_ventana(p.ventana[-30],p.selected_method)
                    if ahora > p.umbral:
                        msg.frame_id = "emg_intention_" + p.selected_method
                        msg.stamp = rospy.Time.now()
                        msg.seq = ahora
                        p.pub_intention.publish(msg)
                        time.sleep(0.1)
            else:
                pass
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        

