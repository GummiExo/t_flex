#!/usr/bin/python
import rospy
import time
#import socket
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PointStamped
from t_flex.msg import IMUData
import numpy as np
import scipy.io as sio
from scipy.signal import butter, lfilter, filtfilt
import math



class MovementIntention(object):
    def __init__(self):
        ''' EMG '''
        self.ventana = []
        self.emgventana = []
        self.umbral = 0
        self.muestras = 500
        self.estadoemg = 'sin'
        self.tiemp = True
        self.detecciones = []
        self.ult = 0
        self.accion_emg = False
        ''' IMU '''
        self.datos = []
        self.imudatos = []
        self.y = []
        self.promedios = []
        self.deteccionesimu = []
        self.tiempi = True
        self.accion = False
        self.estadoimu = 'sin'
        #self.operador = "MEAN"
        self.msg_publisher = Header()
        ''' ROS commands '''
        rospy.init_node('movement_intention_detector', anonymous = True)
        self.initParameters()
        self.initSubscribers()
        topic_name = "/movement_intention_" + self.selected_sensor + "_" + self.selected_method
        self.pub_intention = rospy.Publisher(topic_name, Header, queue_size = 1, latch = False)
        topic_name = "/sound_command_" + self.selected_sensor + "_" + self.selected_method
        self.pub_commands = rospy.Publisher(topic_name, Header, queue_size = 1, latch = False)
        topic_name = "/angle_" + self.selected_sensor + "_" + self.selected_method
        self.pub_angle = rospy.Publisher(topic_name, Header, queue_size = 1, latch = False)
        self.isIMUUpdated = False
        self.isEMGUpdated = False
        self.pub_activation = rospy.Publisher("/enable_device", Bool, queue_size = 1, latch = False)
        #Socket Juego
        #self.mi_socket = socket.socket()
        #self.mi_socket.connect(("192.168.4.9", 3014))

    def initParameters(self):
        self.selected_sensor = rospy.get_param("~sensor","emg") #emg, imu
        self.selected_method = rospy.get_param("~method","mean") #mean, std, var, mean_std, rms
        return

    def initSubscribers(self):
        rospy.Subscriber("/sound_command", Header, self.updateSoundCommands)
        rospy.Subscriber('/imu_data', IMUData, self.updateIMUData)
        if self.selected_sensor == 'emg':
            rospy.Subscriber("/emg_data", PointStamped, self.updateEMGData)
            self.pub_emg_info = rospy.Publisher("/emg_used_data", Header, queue_size = 1, latch = False)
        elif self.selected_sensor == 'imu':
            self.pub_gyro_info = rospy.Publisher("/gyro_y_used_data", Header, queue_size = 1, latch = False)

    def updateSoundCommands(self,commands):
        self.msg_publisher.frame_id = commands.frame_id
        self.msg_publisher.seq = commands.seq
        self.msg_publisher.stamp = rospy.Time.now()
        self.pub_commands.publish(self.msg_publisher)

    def updateIMUData(self,imu_data):
        if imu_data.gyro_y*math.pi/180 > -300*math.pi/180  and imu_data.gyro_y*math.pi/180  < 210*math.pi/180 :
            if self.selected_sensor == 'imu':
                if len(self.datos) >= 50:
                    self.datos = np.delete(self.datos, 0)
                if self.estadoimu == 'con':
                    self.imudatos = np.append(self.imudatos,imu_data.gyro_y*math.pi/180)
                self.datos = np.append(self.datos,imu_data.gyro_y*math.pi/180)

                self.isIMUUpdated = True
                self.msg_publisher.frame_id = str(imu_data.gyro_y*math.pi/180)
                self.msg_publisher.stamp = rospy.Time.now()
                self.msg_publisher.seq = 20 #Angle
                self.pub_gyro_info.publish(self.msg_publisher)
            self.angle_imu = imu_data.angle

    def updateEMGData(self,emg_data):
        if emg_data.point.x < 5:
            if len(self.ventana) >= self.muestras:
                self.ventana = np.delete(self.ventana, 0)
            if self.estadoemg == 'con':
                self.emgventana = np.append(self.emgventana,emg_data.point.x)
            self.ventana = np.append(self.ventana,emg_data.point.x)

            self.isEMGUpdated = True
            self.msg_publisher.frame_id = str(emg_data.point.x)
            self.msg_publisher.stamp = rospy.Time.now()
            self.msg_publisher.seq = 10 #EMG ID
            self.pub_emg_info.publish(self.msg_publisher)
        else:
            self.isEMGUpdated = False

    def butter_filtro_pasabajos(self, ventana):
        frec_corte = 6
        fs = 100
        orden = 4
        nyq = 0.5 * fs
        frecn = frec_corte / nyq
        b, a = butter(orden, frecn, btype='low', analog=False)
        y = lfilter(b, a, ventana)
        return y
    def butter_filtro_pasabandas(self, ventana):
        fc_h = 20
        fc_l = 500
        fs = 1001
        nyq = 0.5 * fs
        b_l, a_l = butter(2,fc_l/nyq,'low')
        b_h, a_h = butter(2,fc_h/nyq,'high')
        y_h = filtfilt(b_h,a_h,ventana)
        y_f = filtfilt(b_l,a_l,y_h)
        y_f = y_f/np.max(np.abs(y_f))
        y_f = np.square(y_f)
        return y_f

    #def deteccion_de_accion(self, y):
    #    j = 0
    #    ca = 0
    #    for x in y:
    #        if x < -0.3:
    #            j = j + 1
    #        if j <= 25:
    #            self.accion = False
    #        else:
    #            self.accion = True
    #		ca = ca + 1
    #    return ca

    def deteccion_de_accion(self, y):
        j = 0
        for x in y:
            if x > -3.0:
                j = j + 1
        if j >= 49:
            self.accion = False
        else:
            self.accion = True
        return

    def calculo_umbral_imu(self):
        if self.isIMUUpdated:
            if len(self.datos) >= 50:
                y = self.butter_filtro_pasabajos(self.datos)

                self.deteccion_de_accion(y)
                self.deteccionesimu = np.append(self.deteccionesimu,True)
                #print(ca)
                #time.sleep(20)
                #print("Hay accion: " + str(accion))
                #accion = True
                if self.accion == True:
                    #print("Hay accion: " + str(accion))
                    if self.selected_method == 'mean':
                        u = np.mean(y)
                    elif self.selected_method == 'std':
                        u = np.std(y)
                    elif self.selected_method == 'var':
                        u = np.var(y)
                    elif self.selected_method == 'mean_std':
                        u = np.mean(y) + (3*np.std(y))
                    elif self.selected_method == 'rms':
                        u = np.sqrt(np.mean(np.array(y)**2))
                    self.umbral = u
                    self.estadoimu = 'con'
                    #self.promedios.append(u)
                    #if len(self.promedios) == 5:
                        #if self.selected_method == 'mean':
                        #self.umbral = np.mean(self.promedios)
                        # elif self.selected_method == 'std':
                        #     self.umbral = np.std(self.promedios)
                        #     print("umbral = " + str(self.umbral))
                        # elif self.selected_method == 'var':
                        #     self.umbral = np.var(self.promedios)
                        # elif self.selected_method == 'mean_std':
                        #     self.umbral = np.mean(self.promedios) + (3*np.std(self.promedios))
                        # elif self.selected_method == 'rms':
                        #     self.umbral = np.sqrt(np.mean(np.array(self.promedios))**2))
                        # else:
                        #     rospy.logerr("Method not selected")
                        #print("Listo el umbral", self.umbral)
            else:
                return
            self.isIMUUpdated = False
        else:
            return

    def calculo_umbral_emg(self):
        if self.isEMGUpdated:
            if ((len(self.ventana) == self.muestras) and (self.estadoemg == 'sin')):
                #self.ventana = self.butter_filtro_pasabandas(self.ventana)
                #self.eav = np.mean(np.abs(np.diff(self.ventana)))
		        #self.promedios = np.append(self.promedios,0)
                self.detecciones = np.append(self.detecciones,True)
                self.ult = self.amplitud(self.ventana)
                if self.selected_method == 'mean':
                    self.umbral = np.mean(self.ventana)
                    self.estadoemg = 'con'
                elif self.selected_method == 'std':
                    self.umbral = np.std(self.ventana)
                    self.estadoemg = 'con'
                elif self.selected_method == 'var':
                    self.umbral = np.var(self.ventana)
                    self.estadoemg = 'con'
                elif self.selected_method == 'mean_std':
                    self.umbral = np.mean(self.ventana) + (3*np.std(self.ventana))
                    self.estadoemg = 'con'
                elif self.selected_method == 'rms':
                    self.umbral = np.sqrt(np.mean(np.array(self.ventana)**2))
                    self.estadoemg = 'con'
                else:
                    rospy.logerr("Method not selected")
                return
            else:
                return
            self.isEMGUpdated = False
        else:
            return
    def amplitud(self,emg):
        amp = np.mean(emg)
	return amp
    def operacion_ventana_imu(self,ventana,tipo):
        j = 0
        for x in ventana:
            if x > -3.0:
                j = j + 1
        if j >= 9:
            self.accion = False
        else:
            self.accion = True

        if tipo == 'mean':
            valor = np.mean(ventana)
            self.msg_seq = 1
        if tipo == 'std':
            valor = np.std(ventana)
            self.msg_seq = 2
        if tipo == 'var':
            valor = np.var(ventana)
            self.msg_seq = 3
        if tipo == "mean_std":
            valor = np.mean(ventana) + (3*np.std(ventana))
            self.msg_seq = 4
        if tipo == "rms":
            valor = np.sqrt(np.mean(np.array(ventana)**2))
            self.msg_seq = 5
        return valor

    def operacion_ventana_emg(self,ventana,tipo):

        if tipo == 'mean':
            valor = np.mean(ventana)
            self.msg_seq = 1
        if tipo == 'std':
            valor = np.std(ventana)
            self.msg_seq = 2
        if tipo == 'var':
            valor = np.var(ventana)
            self.msg_seq = 3
        if tipo == "mean_std":
            valor = np.mean(ventana) + (3*np.std(ventana))
            self.msg_seq = 4
        if tipo == "rms":
            valor = np.sqrt(np.mean(np.array(ventana)**2))
            self.msg_seq = 5
        return valor

    def deteccion_de_accion_emg(self,y):
        j = 0
        self.promedios = np.diff(y)
        for x in self.promedios:
            if x < 0.0:
                j = j + 1
        if j > 0:
            print(j)
            self.accion_emg = False
        else:
            self.accion_emg = True

        return


def main():
    p = MovementIntention()
    rate = rospy.Rate(2000)
    rospy.loginfo("Starting Movement Intention Detector %s",p.selected_sensor)
    msg = Header()
    while not (rospy.is_shutdown()):
        if ((p.selected_sensor == 'imu') or (p.selected_sensor == 'IMU')):
            if p.isIMUUpdated:
                if p.estadoimu == 'sin':
                    p.calculo_umbral_imu()
                else:
                    #rospy.loginfo("IMU Threshold Calculated: %s",p.umbral)
                    #ahora = p.operacion_ventana_imu(p.datos[-10:],p.selected_method)
                    if len(p.imudatos) >= 10:
                        ahora = p.operacion_ventana_imu(p.imudatos,p.selected_method)
                        #print("ahora imu: " + str(ahora))
                        #print ("umbral imu " + str(p.umbral))
                        #print(str(ahora) + "  " + str(p.umbral))
	                #print("IMU UMBRAL   " +  str(p.umbral))
                        if p.accion == True and ahora < p.umbral:
                            p.deteccionesimu = np.append(p.deteccionesimu,False)
                            if np.all(p.deteccionesimu[-3:1]):
                                if p.tiempi == True:
                                    #p.pub_activation.publish(1)
                                    #p.mi_socket.send('jump'.encode())
                                    msg.frame_id = str(ahora) #"imu_intention_" + p.selected_method
                                    detection_time = rospy.Time.now()
                                    msg.stamp = detection_time
                                    msg.seq = 20 + p.msg_seq #Method ID
                                    p.pub_intention.publish(msg)
                                    msg.frame_id = str(p.angle_imu)
                                    msg.stamp = detection_time
                                    msg.seq = 10 + p.msg_seq
                                    p.pub_angle.publish(msg)
                                    p.tiempi = False
                                    timiref = time.time()
                                    #time.sleep(1)
                                if (time.time() - timiref) > 1.0:
                                    #print("YEEEEEEEEEEEEEEEEEEEEEEE!!!!!!!!!!!!!!!!!!")
                                    p.tiempi = True
                            else:
                                p.deteccionesimu = np.append(p.deteccionesimu,True)
                            #time.sleep(0.7)
                        p.imudatos = []
            else:
                pass
        elif ((p.selected_sensor == 'emg') or (p.selected_sensor == 'EMG')):
            if p.isEMGUpdated:
                if p.estadoemg == 'sin':
                    p.calculo_umbral_emg()
                else:
                    #rospy.loginfo("EMG Threshold Calculated: %s", p.umbral)
                    #ahora = p.operacion_ventana_emg(p.ventana[-30:],p.selected_method)

                    if len(p.emgventana) >= 30:
                        #p.emgventana = p.butter_filtro_pasabandas(p.emgventana)
                        ahora = p.operacion_ventana_emg(p.emgventana,p.selected_method)
                        #print("EMG VENTANA   " + str(p.emgventana))
	                #p.deteccion_de_accion_emg(p.emgventana)
                        #print(type(p.emgventana[]))
                        res = p.emgventana[-1:] - p.emgventana[:1]
                        #aver = np.mean(np.abs(np.diff(p.emgventana)))
                        #print("ahora emg: " + str(ahora))
                        #print ("umbral emg " + str(p.umbral))

                        if ahora > p.umbral and res > 0.0 and p.emgventana[-1:] >= 1.1*p.ult:
                            #and p.emgventana[-1:] >= 1.1*p.ult
                            p.detecciones = np.append(p.detecciones,False)
                            #print(p.detecciones)
                            if np.all(p.detecciones[-2:-1]):
                                if p.tiemp == True:
                                    #p.pub_activation.publish(1)
                                    #p.mi_socket.send('jump'.encode())
                                    msg.frame_id = str(ahora) #"emg_intention_" + p.selected_method
                                    detection_time = rospy.Time.now()
                                    msg.stamp = detection_time
                                    msg.seq = 10 + p.msg_seq #Method ID
                                    p.pub_intention.publish(msg)
                                    msg.stamp = detection_time
                                    msg.seq = 10 + p.msg_seq
                                    p.pub_angle.publish(msg)
                                    p.tiemp = False
                                    timeref = time.time()
                                    #time.sleep(1)
                                    #print("ENTRE")
                                if (time.time() - timeref) > 1.0:
                                    #print("YAAAAAAAAAAAAAAAAAAAAAAA!!!!!!!!!!!!!!!!!!")
			                        p.tiemp = True
                                    #time.sleep(0.7)
                        else:
                             p.detecciones = np.append(p.detecciones,True)
                        p.emgventana = []
            else:
                pass
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
