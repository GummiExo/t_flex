#! /usr/bin/python
# -*- coding: utf-8 -*-

import tornado
import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
from server import Server

import rospkg

rospack = rospkg.RosPack()
package_directory = rospack.get_path('t_flex')
#Tornado Folder Paths
settings = dict(
	template_path = package_directory + "/WebController",
	static_path = package_directory + "/WebController/src",
	)
#Server Port
PORT = 3012

server = Server()
server.initialization()

class MainHandler(tornado.web.RequestHandler):
	def get(self):
		print ("[HTTP](MainHandler) User Connected.")
		self.render("index.html")

class CalibrationHandler(tornado.web.RequestHandler):
	def get(self):
		print ("[HTTP](CalHandler) User Connected.")
		self.render("calibration.html", messages=WSHandler)

class TherapyHandler(tornado.web.RequestHandler):
	def get(self):
		print ("[HTTP](CalHandler) User Connected.")
		self.render("therapy.html", messages=WSHandler)

class AssistanceHandler(tornado.web.RequestHandler):
	def get(self):
		print ("[HTTP](CalHandler) User Connected.")
		self.render("assistance.html", messages=WSHandler)

class ConfigureHandler(tornado.web.RequestHandler):
	def get(self):
		print ("[HTTP](CalHandler) User Connected.")
		self.render("configure.html", messages=WSHandler)

class BCIHandler(tornado.web.RequestHandler):
	def get(self):
		print ("[HTTP](CalHandler) User Connected.")
		self.render("bci.html", messages=WSHandler)

class WSHandler(tornado.websocket.WebSocketHandler):
	def open(self):
		print ('[WS] Connection was opened.')
		# self.write_message("Conectado")

	def on_message(self, message):
		print ('[WS] Incoming message:'), message
		try:
			pos_args = message.index(' ')
			command = message[0:pos_args]
			args = message[pos_args+1:len(message)]
		except:
			command = message
		print ("Received Command: " + command)
		if command == 'activate_device':
			is_done, msg = server.run_dynamixel_controllers()
		elif command  == 'angle_calibration':
			is_done, msg = server.run_angle_calibration()
		elif command == 'stiffness_calibration':
			is_done, msg = server.run_stiffness_calibration()
		elif command == 'stop_angle_calibration':
			is_done, msg = server.stop_angle_calibration()
		elif command == 'stop_stiffness_calibration':
			is_done, msg = server.stop_stiffness_calibration()
		elif command == 'start_therapy':
			args = args.split(' ')
			args = list(map(str,args))
			num_rep = args[0]
			freq = args[1]
			vel = args[2]
			print ("Received Args: Repetitions = " + str(num_rep) + " Frequency = " + str(freq) + " Velocity = " + str(vel))
			is_done, msg = server.start_therapy(repetitions = num_rep, frequency = freq, velocity = vel)
		elif command == 'stop_therapy':
			is_done, msg = server.stop_therapy()
		elif command == 'start_assistance':
			args = args.split(' ')
			args = list(map(str,args))
			patient_name = ''
			for i in range (0,len(args)-2):
				if i == 0:
					patient_name = patient_name + args[0]
				else:
					patient_name = patient_name + '_' + args[i]
			if (('.' in patient_name) or ('/' in patient_name) or ('-' in patient_name)):
				is_done, msg = False, ("Los caracteres ingresados en el nombre del paciente no son aceptados. \nIngrese nuevamente el nombre")
			else:
				time_assistance = args[-1]
				algorithm = args[-2]
				print ("Received Args: Patient: " + patient_name + " ,Algorithm = " + algorithm + " ,Time = " + time_assistance)
				is_done, msg = server.start_assistance(time_assistance = time_assistance, patient_name = patient_name, algorithm = algorithm)
		elif command == 'open_port':
			is_done, msg = server.open_port()
		elif command == 'stop_assistance':
			is_done, msg = server.stop_assistance()
		elif command == 'start_therapy_bci':
			is_done, msg = server.start_therapy_bci()
		elif command == 'open_terminal':
			is_done, msg = server.open_terminal()
		elif command == 'close_terminal':
			is_done, msg = server.close_terminal()
		elif command == 'exit':
			is_done, msg = server.exit()
		elif command == 'shutdown':
			is_done, msg = server.shutdown()
		else:
			print ("Error: Unknown command")
		try:
			if is_done:
				self.write_message(msg)
			else:
				self.write_message("¡ERROR!\n" + msg)
		except:
			pass

	def on_close(self):
		print ('[WS] Connection was closed.')
		#self.write_message("Conexion Terminada")

#Applications
menu_application = tornado.web.Application([
	(r'/', MainHandler),
	(r'/calibration.html', CalibrationHandler),
	(r'/therapy.html', TherapyHandler),
	(r'/assistance.html', AssistanceHandler),
	(r'/configure.html', ConfigureHandler),
	(r'/bci.html', BCIHandler),
	(r'/ws', WSHandler),
	("/src/(.*)",tornado.web.StaticFileHandler, {"path": package_directory + "/WebController/src"},),
	], **settings)

if __name__ == "__main__":
    try:
        menu = tornado.httpserver.HTTPServer(menu_application)
        menu.listen(PORT)
        print ("Tornado Server started on ports: " + str(PORT))
        main_loop = tornado.ioloop.IOLoop.instance()
        main_loop.start()

    except Exception as e:
		print ("Exception triggered - Tornado Server stopped.")
		print (e)
