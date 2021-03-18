#!/usr/bin/python
import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PointStamped
import serial

class electromiografia(object):
    
    def __init__(self):
        ''' Variables '''
        self.dato = 0
        
        ''' Serial port configuration '''
        self.ser=serial.Serial()
        self.ser.baudrate=115200
        self.ser.port = "/dev/ttyACM0"
        self.kill_therapy = False
        if self.ser.isOpen():
            self.ser.close()
        self.ser.open()
        
        ''' ROS commands '''
        rospy.init_node('emg_acquisition', anonymous = True)
        rospy.Subscriber("/kill_emg_sensor", Bool, self.updateFlagAcquisition)
        self.emg_data_pub = rospy.Publisher("/emg_data", PointStamped, queue_size = 1, latch = False)

	
    def updateFlagAcquisition(self, flag):
        self.kill_therapy = flag.data
        
    def calculo_umbral(self,tipo):
        if (len(self.ventana) == self.muestras) and (self.estado == 'sin'):
            if tipo == 'MEAN':
                self.umbral = np.mean(self.ventana)
                self.estado = 'con'
                print('LISTO')
            if tipo == 'STD':
                self.umbral = np.std(self.ventana)
                self.estado = 'con'
                print('LISTO')
            if tipo == 'VAR':
                self.umbral = np.var(self.ventana)
                self.estado = 'con'
                print('LISTO')
            self.ventana = []
        else:
            if self.estado == 'sin':
                print('CALCULANDO')

    def operacion_ventana(self,ventana,tipo):
        if tipo == 'MEAN':
            valor = np.mean(ventana)
        if tipo == 'STD':
            valor = np.std(ventana)
        if tipo == 'VAR':
            valor = np.var(ventana)
        return valor

    def arreglo(self,dato):
        dato = dato[:4]
        if dato.count('.')>1:
            dato = dato.rstrip('.')
        dato = float(dato)
        return dato

    def verificacion(self,ahora,veri,x):
        if ahora > self.umbral:
            if ((self.ventana[29]-self.ventana[0]) > 0) and (veri == True):
                print('AQUI SI',self.umbral, ahora)
                x.append(len(self.datos)-30)
                veri = False
        else:
            veri = True
            print('AQUI NO',self.umbral, ahora)
        return veri,x


        
    def recibir_dato(self):
        self.dato = self.ser.readline().decode().strip()
        if len(self.dato)>=4:
            self.dato = self.arreglo(self.dato)
        

def main():
    s = electromiografia()
    rate = rospy.Rate(1000) # Sensor Frequency 1000 Hz 
    rospy.loginfo("Starting AT04001 EMG sensor acquisition")
    while not (rospy.is_shutdown()):
        try:
            if s.kill_therapy:
                break
            s.recibir_dato()
            if isinstance(s.dato, float):
                msg = PointStamped()
                msg.point.x = s.dato
                msg.header.frame_id = "/emg_data"
                msg.header.stamp = rospy.Time.now()
                s.emg_data_pub.publish(msg)
        except Exception as e:
            print(e)
            rospy.logwarn("EMG Sensor Disconnected")
        rate.sleep()
    rospy.loginfo("Acquisition Finished")


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
