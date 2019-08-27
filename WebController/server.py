#! /usr/bin/python
# -*- coding: utf-8 -*-

import os
import threading
import time
import subprocess

class Server(object):
	def __init__(self):
		''' Topics Name '''
		motor_state_topic_frontal = '/motor_states/frontal_tilt_port'
		motor_state_topic_posterior = '/motor_states/posterior_tilt_port'
		frontal_command_topic = '/tilt1_controller/command'
		posterior_command_topic = '/tilt2_controller/command'
		flag_angle_calibration = '/t_flex/kill_angle_calibration'
		flag_stiffness_calibration = '/t_flex/kill_stiffness_calibration'
		flag_therapy = '/t_flex/kill_therapy'
		gait_phases_detection_topic = '/t_flex/gait_phase_detection'
		imu_data_topic = '/t_flex/imu_data'
		flag_gait_assistance = '/t_flex/kill_gait_assistance'
		flag_speed = '/t_flex/update_speed'
		flag_enable_device = '/t_flex/enable_device'

	def rostopic_list(self):
		rostopic_list = subprocess.Popen(['rostopic', 'list',], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		stdout, stderr = rostopic_list.communicate()
		return stdout

	def run_dynamixel_controllers(self):
		topics = self.rostopic_list()
		if not ((motor_state_topic_frontal in topics) and (motor_state_topic_posterior in topics)):
			rd =threading.Thread(target=self.launchDynamixelController)
			print("Running Dynamixel Controllers ID's: 1 y 2 , Port: ttyUSB0")
			rd.start()
			time.sleep(10)
			os.system("roslaunch t_flex start_position_controller.launch")
			topics = self.rostopic_list()
			if ((motor_state_topic_frontal in topics) and (motor_state_topic_posterior in topics) and (frontal_command_topic in topics) and (posterior_command_topic in topics)):
				return True, ("Controladores Inicializados")
			else:
				return False, ("El controlador no encuentra motores, revise conexiones")
		else:
			return False, ("Los motores ya están activados")

	def run_angle_calibration(self):
		topics = self.rostopic_list()
		if ((motor_state_topic_frontal in topics) and (motor_state_topic_posterior in topics)):
			if not ((flag_angle_calibration in topics) or (flag_stiffness_calibration in topics)):
				ac = threading.Thread(target=self.runAngleCalibration)
				ac.start()
				time.sleep(5)
				topics = self.rostopic_list()
				if (flag_angle_calibration in topics):
					return True, ("Calibración de Ángulo Iniciada")
				else:
					return False, ("No se pudo iniciar calibración, intente de nuevo")
			else:
				return False, ("Calibración corriendo, porfavor termine la calibración actualmente en curso")
		else:
			return False, ("Los motores no se encuentran activados")

	def run_stiffness_calibration(self):
		topics = self.rostopic_list()
		if ((motor_state_topic_frontal in topics) and (motor_state_topic_posterior in topics)):
			if not ((flag_angle_calibration in topics) or (flag_stiffness_calibration in topics)):
				sc = threading.Thread(target=self.runStiffnessCalibration)
				sc.start()
				time.sleep(5)
				topics = self.rostopic_list()
				if (flag_stiffness_calibration in topics):
					return True, ("Calibración de Rigidez Iniciada")
				else:
					return False, ("No se pudo iniciar calibración, intente de nuevo")
			else:
				return False, ("Calibración corriendo, porfavor termine la calibración actualmente en curso")
		else:
			return False, ("Los motores no se encuentran activados")

	def stop_angle_calibration(self):
		topics = self.rostopic_list()
		if (flag_angle_calibration in topics):
			os.system("rostopic pub -1 /t_flex/kill_angle_calibration std_msgs/Bool True")
			time.sleep(1)
			return True, ("Calibración de Ángulo Detenida")
		else:
			return False,("Esta calibración no esta ejecutandose")

	def stop_stiffness_calibration(self):
		topics = self.rostopic_list()
		if (flag_stiffness_calibration in topics):
			os.system("rostopic pub -1 /t_flex/kill_stiffness_calibration std_msgs/Bool True")
			time.sleep(1)
			return True, ("Calibración de Rigidez Detenida")
		else:
			return False, ("Esta calibración no esta ejecutandose")

	def start_therapy(self,repetitions,frequency,velocity):
		topics = self.rostopic_list()
		if ((motor_state_topic_frontal in topics) and (motor_state_topic_posterior in topics)):
			if not ((flag_therapy in topics) or (gait_phases_detection_topic in topics) or (imu_data_topic in topics)):
				st = threading.Thread(target=self.runTherapy, args=(repetitions, frequency, velocity))
				st.start()
				time.sleep(3)
				topics = self.rostopic_list()
				if (flag_therapy in topics):
					return True, ("Terapia Iniciada")
				else:
					return False, ("No se pudo iniciar terapia, Iintente nuevamente")
			else:
				return False, ("Se está ejecutando uno de los modos de operación")
		else:
			return False, ("Los motores no se encuentran activados")

	def stop_therapy(self):
		topics = self.rostopic_list()
		if (flag_therapy in topics):
			os.system("rostopic pub -1 /t_flex/kill_therapy std_msgs/Bool True")
			time.sleep(2)
			return True, ("Terapia Detenida")
		else:
			return False, ("No se esta ejecutando el modo de terapia")

	def start_assistance(self,time_assistance):
		topics = self.rostopic_list()
		if ((motor_state_topic_frontal in topics) and (motor_state_topic_posterior in topics)):
			if not ((flag_therapy in topics) or (gait_phases_detection_topic in topics) or (imu_data_topic in topics)):
				print type(time_assistance)
				st = threading.Thread(target=self.launchAssistance, args=(time_assistance))
				st.start()
				time.sleep(8)
				topics = self.rostopic_list()
				if (flag_gait_assistance in topics) and (gait_phases_detection_topic in topics) and (imu_data_topic in topics):
					return True, ("Asistencia Iniciada")
				else:
					return False, ("No se pudo iniciar asistencia, Intente nuevamente")
			else:
				return False, ("Se está ejecutando uno de los modos de operación")
		else:
			return False, ("Los motores no se encuentran activados")

	def stop_assistance(self):
		topics = self.rostopic_list()
		if (flag_gait_assistance in topics) and (gait_phases_detection_topic in topics) and (imu_data_topic in topics):
			os.system("rostopic pub -1 /t_flex/kill_gait_assistance std_msgs/Bool True")
			time.sleep(3)
			return True, ("Terapia Detenida")
		else:
			return False, ("No se esta ejecutando el modo de terapia")

	def open_port(self):
		topics = self.rostopic_list()
		if ('/rosout' in topics):
			ports = subprocess.Popen(['netstat', '-au',], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			stdout, stderr = ports.communicate()
			udp_ports = stdout
			if not ('192.168.4.1:3014' in udp_ports):
				us = threading.Thread(target=self.runUDPServer)
				us.start()
				time.sleep(2.5)
				ports = subprocess.Popen(['netstat', '-au',], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
				stdout, stderr = ports.communicate()
				udp_ports = stdout
				if ('192.168.4.1:3014' in udp_ports):
					return True, ("Escuchando en 192.168.4.1 por puerto 3014\nEnvie 1 para activar y 0 para desactivar")
				else:
					return False, ("No está activo el servidor, intente nuevamente")
			else:
				return False, ("El servidor ya esta ejecutandose")
		else:
			return False, ("Roscore no está corriendo, active los motores antes de abrir el puerto")

	def start_therapy_bci(self):
		topics = self.rostopic_list()
		if ((motor_state_topic_frontal in topics) and (motor_state_topic_posterior in topics)):
			if not ((flag_therapy in topics) or (gait_phases_detection_topic in topics) or (imu_data_topic in topics)):
				st_bci = threading.Thread(target=self.runTherapyBCI)
				st_bci.start()
				time.sleep(3)
				topics = self.rostopic_list()
				if ((flag_therapy in topics) and (flag_speedin topics) and (flag_enable_device in topics)):
					return True, ("Terapia Iniciada")
				else:
					return False, ("No se pudo iniciar terapia, Iintente nuevamente")
			else:
				return False, ("Se está ejecutando uno de los modos de operación")
		else:
			return False, ("Los motores no se encuentran activados")

	def open_terminal(self):
		ot = threading.Thread(target=self.runSSH)
		ot.start()
		return True, ("Puerto 3013 Abierto\nHostname: 192.168.4.1\nUser: pi\nPassword: admin")

	def close_terminal(self):
		os.system("fuser -k -n tcp 3013")
		return True, ("Puerto Cerrado")

	def exit(self):
		topics = self.rostopic_list()
		if ('/rosout' in topics):
			os.system("pkill -f ros")
			return True, ("Todos los procesos han finalizado")
		else:
			return False, ("No estan corriendo procesos en ROS")

	''' Threadings '''
	def launchDynamixelController(self):
		os.system("sudo chmod 777 /dev/ttyUSB0")
		os.system("sudo chmod 777 /dev/ttyUSB1")
		os.system("roslaunch t_flex dynamixel_controllers.launch")

	def runAngleCalibration(self):
		os.system("rosrun t_flex angle_calibration.py")

	def runStiffnessCalibration(self):
		os.system("rosrun t_flex stiffness_calibration.py")

	def runTherapy(self,repetitions,frequency,velocity):
		os.system("rosrun t_flex therapy.py " + repetitions + " " + frequency + " " + velocity)

	def launchAssistance(self,*time_assistance):
		os.system("roslaunch t_flex gait_assistance.launch time:=" + str("".join(time_assistance)))

	def runUDPServer(self,*port):
		os.system("rosrun t_flex udp_listener.py -h 192.168.4.1 -p 3014")

	def runTherapyBCI(self):
		os.system("rosrun t_flex therapy_with_flags.py " + repetitions + " " + frequency + " " + velocity)

	def runSSH(self):
		os.system("wssh --address='192.168.4.1' --port=3013")
