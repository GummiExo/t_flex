#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
sys.path.append('/opt/ros/melodic/share')
sys.path.append('/home/pi/catkin_ws/devel/lib/python2.7/dist-packages')
sys.path.append('/home/pi/catkin_ws/src')
import rospy, roslaunch, rosnode, rostopic
from std_msgs.msg import Bool
import os
import threading
import time
import subprocess
import datetime

class Server(object):
	def __init__(self):
		''' Topics Name '''
		self.motor_state_topic_frontal = '/motor_states/frontal_tilt_port'
		self.motor_state_topic_posterior = '/motor_states/frontal_tilt_port'
		self.frontal_command_topic = '/tilt1_controller/command'
		self.posterior_command_topic = '/tilt2_controller/command'
		self.flag_angle_calibration = '/t_flex/kill_angle_calibration'
		self.flag_stiffness_calibration = '/t_flex/kill_stiffness_calibration'
		self.flag_therapy = '/t_flex/kill_therapy'
		self.gait_phases_detection_topic = '/gait_phase_detection'
		self.imu_data_topic = '/imu_data'
		self.flag_gait_assistance = '/kill_gait_assistance'
		self.flag_speed = '/t_flex/update_speed'
		self.flag_enable_device = '/t_flex/enable_device'
		''' Nodes Name '''
		self.angle_calibration_node = '/angle_calibration'
		self.stiffness_calibration_node = '/stiffness_calibration'
		self.therapy_node = '/therapy'
		self.gait_assistance_node = '/gait_assistance'
		self.gait_phases_detection_node = '/threshold_gait_detection_node'
		''' Others '''
		self.imu_address = '29'

	def initialization(self):
		''' ROS config '''
		self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(self.uuid)
		self.package_tflex = 't_flex'
		self.config_node = roslaunch.config.ROSLaunchConfig()
		self.machine = roslaunch.core.local_machine()
		self.config_node.add_machine(m=self.machine)
		self.run_id = rospy.get_param('/run_id')
		''' Launchers '''
		self.launch_dynamixel_controller = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/pi/catkin_ws/src/t_flex/launch/dynamixel_controller.launch"])
		self.launch_position_controller = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/pi/catkin_ws/src/t_flex/launch/start_position_controller.launch"])
		self.launch_angle_calibration = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/pi/catkin_ws/src/t_flex/launch/angle_calibration.launch"])
		self.launch_stiffness_calibration = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/pi/catkin_ws/src/t_flex/launch/stiffness_calibration.launch"])
		cli_args = ['/home/pi/catkin_ws/src/t_flex/launch/therapy.launch','rep:=0','freq:=0.0','velocity:=0']
		roslaunch_args = cli_args[1:]
		self.roslaunch_therapy_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
		self.launch_therapy = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_therapy_file)
		cli_args = ['/home/pi/catkin_ws/src/t_flex/launch/gait_assistance_threshold.launch','time:=0']
		roslaunch_args = cli_args[1:]
		self.roslaunch_assistance_threshold_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
		self.launch_assistance_threshold = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_assistance_threshold_file)
		cli_args = ['/home/pi/catkin_ws/src/t_flex/launch/gait_assistance_machine_learning.launch','time:=0']
		roslaunch_args = cli_args[1:]
		self.roslaunch_assistance_machine_learning_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
		self.launch_assistance_machine_learning = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_assistance_machine_learning_file)
		# ''' Nodes '''
		# self.node_angle_calibration = roslaunch.core.Node(package=self.package_tflex, node_type='angle_calibration.py', name='angle_calibration')
		# self.node_stiffness_calibration = roslaunch.core.Node(package=self.package_tflex, node_type='stiffness_calibration.py', name='stiffness_calibration')
		# self.node_therapy = roslaunch.core.Node(package=self.package_tflex, node_type='therapy.py', name='therapy')
		# ''' Launch function for nodes'''
		# self.config_angle_calibration_node = self.config_node
		# self.config_angle_calibration_node.add_node(self.node_angle_calibration)
		# self.launch_angle_calibration = roslaunch.launch.ROSLaunchRunner(config=self.config_angle_calibration_node,run_id=self.run_id)
		# self.config_stiffness_calibration_node = self.config_node
		# self.config_stiffness_calibration_node.add_node(self.node_stiffness_calibration)
		# self.launch_stiffness_calibration = roslaunch.launch.ROSLaunchRunner(config=self.config_stiffness_calibration_node,run_id=self.run_id)
		# self.config_therapy_node = self.config_node
		# self.config_therapy_node.add_node(self.node_therapy)
		# self.launch_therapy = roslaunch.launch.ROSLaunchRunner(config=self.config_therapy_node,run_id=self.run_id)
		''' Publishers '''
		#self.pub_angle_calibration, msg = rostopic.create_publisher('/t_flex/angle_calibration', "Bool", 0)

	def flatten(self,seq):
		l = []
		for elt in seq:
			t = type(elt)
			if t is tuple or t is list:
				for elt2 in self.flatten(elt):
					l.append(elt2)
			else:
				l.append(elt)
		return l

	def rostopic_list(self):
		topic_list = rostopic.get_topic_list()
		topics = self.flatten(topic_list)
		return topics

	def rosnode_list(self):
		nodes = rosnode.get_node_names()
		return nodes

	def run_dynamixel_controllers(self):
		topics = self.rostopic_list()
		if not ((self.motor_state_topic_frontal in topics) and (self.motor_state_topic_posterior in topics)):
			os.system("sudo /home/pi/catkin_ws/src/t_flex/config.sh")
			self.launch_dynamixel_controller.start()
			time.sleep(10)
			rospy.loginfo("Dynamixel Started")
			print("Running Dynamixel Controllers ID's: 1 and 2 , Port: ttyUSB0")
			topics = self.rostopic_list()
			if ((self.motor_state_topic_frontal in topics) and (self.motor_state_topic_posterior in topics)):
				print("Motores Found, starting position controller...")
				self.launch_position_controller.start()
				time.sleep(5)
				topics = self.rostopic_list()
				if ((self.motor_state_topic_frontal in topics) and (self.motor_state_topic_posterior in topics) and (self.frontal_command_topic in topics) and (self.posterior_command_topic in topics)):
					return True, ("Controladores Inicializados")
				else:
					self.launch_position_controller.shutdown()
					self.launch_dynamixel_controller.shutdown()
					return False, ("Error inicializando controlador de posicion, revise conexiones entre Motores")
			else:
				self.launch_position_controller.shutdown()
				self.launch_dynamixel_controller.shutdown()
				return False, ("El controlador no encuentra motores, revise conexiones")
		else:
			return False, ("Los motores ya están activados")

	def run_angle_calibration(self):
		topics = self.rostopic_list()
		if ((self.motor_state_topic_frontal in topics) and (self.motor_state_topic_posterior in topics)):
			nodes = self.rosnode_list()
			if not ((self.angle_calibration_node in nodes) or (self.stiffness_calibration_node in nodes)):
				#self.launch_angle_calibration.launch_node(self.node_angle_calibration) #Running from Node File
				#self.launch_angle_calibration.spin_once() #Running from Node File
				self.launch_angle_calibration.start() #Running from Launch File
				time.sleep(5)
				nodes = self.rosnode_list()
				if (self.angle_calibration_node in nodes):
					return True, ("Calibración de Ángulo Iniciada")
				else:
					return False, ("No se pudo iniciar calibración, intente de nuevo")
			else:
				return False, ("Calibración corriendo, porfavor termine la calibración actualmente en curso")
		else:
			return False, ("Los motores no se encuentran activados")

	def run_stiffness_calibration(self):
		topics = self.rostopic_list()
		if ((self.motor_state_topic_frontal in topics) and (self.motor_state_topic_posterior in topics)):
			nodes = self.rosnode_list()
			if not ((self.angle_calibration_node in nodes) or (self.stiffness_calibration_node in nodes)):
				#self.launch_stiffness_calibration.launch_node(self.node_stiffness_calibration) #Running from Node file
				#self.launch_stiffness_calibration.spin_once() #Running from Node File
				self.launch_stiffness_calibration.start() #Running from Launch File
				time.sleep(5)
				nodes = self.rosnode_list()
				if (self.stiffness_calibration_node in nodes):
					return True, ("Calibración de Rigidez Iniciada")
				else:
					return False, ("No se pudo iniciar calibración, intente de nuevo")
			else:
				return False, ("Calibración corriendo, porfavor termine la calibración actualmente en curso")
		else:
			return False, ("Los motores no se encuentran activados")

	def stop_angle_calibration(self):
		nodes = self.rosnode_list()
		if (self.angle_calibration_node in nodes):
			os.system("rostopic pub -1 " + self.flag_angle_calibration + " std_msgs/Bool True")
			time.sleep(1)
			nodes = self.rosnode_list()
			if not (self.angle_calibration_node in nodes):
				return True, ("Calibración de Ángulo Detenida")
			else:
				return False, ("No se pudo detener la calibracion, intente nuevamente")
		else:
			return False,("Esta calibración no esta ejecutandose")

	def stop_stiffness_calibration(self):
		nodes = self.rosnode_list()
		if (self.stiffness_calibration_node in nodes):
			os.system("rostopic pub -1 " + self.flag_stiffness_calibration + " std_msgs/Bool True")
			time.sleep(1)
			nodes = self.rosnode_list()
			if not (self.stiffness_calibration_node in nodes):
				return True, ("Calibración de Rigidez Detenida")
			else:
				return False, ("No se pudo detener la calibracion, intente nuevamente")
		else:
			return False, ("Esta calibración no esta ejecutandose")

	def start_therapy(self,repetitions,frequency,velocity):
		topics = self.rostopic_list()
		if ((self.motor_state_topic_frontal in topics) and (self.motor_state_topic_posterior in topics)):
			nodes = self.rosnode_list()
			if not ((self.therapy_node in nodes) or (self.gait_phases_detection_topic in topics) or (self.imu_data_topic in topics)):
				cli_args = ['/home/pi/catkin_ws/src/t_flex/launch/therapy.launch','rep:=0','freq:=0.0','velocity:=0']
				args1 = 'rep:=' + str(repetitions)
				args2 = 'freq:=' + str(frequency)
				args3 = 'velocity:=' + str(velocity)
				#self.node_therapy = roslaunch.core.Node(package=self.package_tflex, node_type='therapy.py', name='therapy', args=args) #Running from Node File
				#self.launch_therapy.launch_node(self.node_therapy) #Running from Node File
				#self.launch_therapy.spin_once() #Running from Node File
				cli_args = ['/home/pi/catkin_ws/src/t_flex/launch/therapy.launch',args1,args2,args3] # Running from Launch File
				roslaunch_args = cli_args[1:] # Running from Launch File
				self.roslaunch_therapy_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)] # Running from Launch File
				self.launch_therapy = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_therapy_file) # Running from Launch File
				self.launch_therapy.start() # Running from Launch File
				time.sleep(3)
				nodes = self.rosnode_list()
				if (self.therapy_node in nodes):
					return True, ("Terapia Iniciada")
				else:
					return False, ("No se pudo iniciar terapia, intente nuevamente")
			else:
				return False, ("Se está ejecutando uno de los modos de operación")
		else:
			return False, ("Los motores no se encuentran activados")

	def stop_therapy(self):
		nodes = self.rosnode_list()
		if (self.therapy_node in nodes):
			os.system("rostopic pub -1 " + self.flag_therapy + " std_msgs/Bool True")
			time.sleep(1)
			nodes = self.rosnode_list()
			if not (self.therapy_node in nodes):
				return True, ("Terapia Detenida")
			else:
				return False, ("No se pudo detener el modo terapia, intente nuevamente")
		else:
			return False, ("No se esta ejecutando el modo de terapia")

	def start_assistance(self,time_assistance,patient_name,algorithm):
		topics = self.rostopic_list()
		if ((self.motor_state_topic_frontal in topics) and (self.motor_state_topic_posterior in topics)):
		#if True:
			i2c_bus = subprocess.Popen(['i2cdetect', '-y', '3',], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			stdout, stderr = i2c_bus.communicate()
			devices_connected = stdout
			if True:
			#if (self.imu_address in devices_connected):
				#if not ((self.flag_therapy in topics) or (self.gait_phases_detection_topic in topics) or (self.imu_data_topic in topics)):
				if not ((self.flag_gait_assistance in topics)):
					args = 'time:=' + str(time_assistance)
					print(args)
					if (algorithm == 'threshold'):
						cli_args = ['/home/pi/catkin_ws/src/t_flex/launch/gait_assistance_threshold.launch',args]
						roslaunch_args = cli_args[1:]
						self.roslaunch_assistance_threshold_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
						self.launch_assistance_threshold = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_assistance_threshold_file)
						self.launch_assistance_threshold.start()
					elif (algorithm == 'machine_learning'):
						cli_args = ['/home/pi/catkin_ws/src/t_flex/launch/gait_assistance_machine_learning.launch',args]
						roslaunch_args = cli_args[1:]
						self.roslaunch_assistance_machine_learning_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
						self.launch_assistance_machine_learning = roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_assistance_machine_learning_file)
						self.launch_assistance_machine_learning.start()
					time.sleep(8)
					topics = self.rostopic_list()
					if ((self.flag_gait_assistance in topics) and (self.gait_phases_detection_topic in topics) and (self.imu_data_topic in topics)):
					#if True:
						date = datetime.datetime.now()
						record = threading.Thread(target=self.recordAssistance, args=(patient_name, date))
						record.start()
						kill_record = threading.Thread(target=self.killRecordDetection)
						kill_record.start()
						return True, ("Asistencia Iniciada - Grabando datos de " + patient_name)
					else:
						os.system("rostopic pub -1 /kill_gait_assistance std_msgs/Bool True")
						time.sleep(4)
						return False, ("No se pudo iniciar asistencia, Intente nuevamente")
				else:
					return False, ("Se está ejecutando uno de los modos de operación")
			else:
				return False, ("No se encuentra conectada la IMU")
		else:
			return False, ("Los motores no se encuentran activados")

	def stop_assistance(self):
		topics = self.rostopic_list()
		if (self.flag_gait_assistance in topics) and (self.gait_phases_detection_topic in topics) and (self.imu_data_topic in topics):
			os.system("rostopic pub -1 /kill_gait_assistance std_msgs/Bool True")
			try:
				os.system("rosnode kill /threshold_gait_detection_node")
			except:
				pass
			try:
				os.system("rosnode kill /real_time_gait_phase_det")
			except:
				pass
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
		if ((self.motor_state_topic_frontal in topics) and (self.motor_state_topic_posterior in topics)):
			if not ((self.flag_therapy in topics) or (self.gait_phases_detection_topic in topics) or (self.imu_data_topic in topics)):
				st_bci = threading.Thread(target=self.runTherapyBCI)
				st_bci.start()
				time.sleep(3)
				topics = self.rostopic_list()
				if ((self.flag_therapy in topics) and (self.flag_speed in topics) and (self.flag_enable_device in topics)):
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

	def shutdown(self):
		os.system("sudo shutdown now")

	''' Threadings '''
	def launchDynamixelController(self):
		os.system("sudo chmod 777 /dev/ttyUSB0")
		os.system("sudo chmod 777 /dev/ttyUSB1")
		os.system("roslaunch t_flex dynamixel_controller.launch")

	def runAngleCalibration(self):
		os.system("rosrun t_flex angle_calibration.py")

	def runStiffnessCalibration(self):
		os.system("rosrun t_flex stiffness_calibration.py")

	def runTherapy(self,repetitions,frequency,velocity):
		os.system("rosrun t_flex therapy.py " + repetitions + " " + frequency + " " + velocity)

	def launchAssistanceThreshold(self,*time_assistance):
		os.system("roslaunch t_flex gait_assistance_threshold.launch time:=" + str("".join(time_assistance)))

	def launchAssistanceMachineLearning(self,*time_assistance):
		os.system("roslaunch t_flex gait_assistance_machine_learning.launch time:=" + str("".join(time_assistance)))

	def recordAssistance(self,patient,date):
		home = os.path.expanduser('~/bags')
		os.chdir(home)
		os.system("rosbag record -a -O " + patient + "_" + str(date.year) + "_" + str(date.month) + "_" + str(date.day) + "_" + str(date.hour) + "_" + str(date.minute) + "_" + str(date.second)	)

	def killRecordDetection(self):
		while True:
			nodes = self.rosnode_list()
			if not (self.gait_assistance_node in nodes):
				try:
					os.system("rosnode kill /threshold_gait_detection_node")
				except:
					pass
				try:
					os.system("rosnode kill /real_time_gait_phase_det")
				except:
					pass
				initial_pos = nodes.find('/record')
				final_pos = initial_pos + nodes[initial_pos::].find('\n')
				os.system("rosnode kill " + nodes[initial_pos:final_pos])
				break;
			else:
				time.sleep(0.5)

	def runUDPServer(self,*port):
		os.system("rosrun t_flex udp_listener.py -h 192.168.4.1 -p 3014")

	def runTherapyBCI(self):
		os.system("rosrun t_flex therapy_with_flags.py " + repetitions + " " + frequency + " " + velocity)

	def runSSH(self):
		os.system("wssh --address='192.168.4.1' --port=3013")
