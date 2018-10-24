#! /usr/bin/python
# -*- coding: utf-8 -*-

import tornado
import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
from server import Server

#Tornado Folder Paths
settings = dict(
	template_path = "/home/pi/WebController",
	static_path = "/home/pi/WebController/src",
	)
#Server Port
PORT = 3012

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
			is_done, msg = Server().run_dynamixel_controllers()
		elif command  == 'angle_calibration':
			is_done, msg = Server().run_angle_calibration()
		elif command == 'stiffness_calibration':
			is_done, msg = Server().run_stiffness_calibration()
		elif command == 'stop_angle_calibration':
			is_done, msg = Server().stop_angle_calibration()
		elif command == 'stop_stiffness_calibration':
			is_done, msg = Server().stop_stiffness_calibration()
		elif command == 'start_therapy':
			args = args.split(' ')
			args = list(map(str,args))
			num_rep = args[0]
			freq = args[1]
			vel = args[2]
			print ("Received Args: Repetitions = " + str(num_rep) + " Frequency = " + str(freq) + " Velocity = " + str(vel))
			is_done, msg = Server().start_therapy(repetitions = num_rep, frequency = freq, velocity = vel)
		elif command == 'stop_therapy':
			is_done, msg = Server().stop_therapy()
		elif command == 'start_assistance':
			time_assistance = args
			print ("Received Args: Time = " + time_assistance)
			is_done, msg = Server().start_assistance(time_assistance = time_assistance)
		elif command == 'open_port':
			is_done, msg = Server().open_port()
		elif command == 'stop_assistance':
			is_done, msg = Server().stop_assistance()
		elif command == 'start_therapy_bci':
			is_done, msg = Server().start_therapy_bci()
		elif command == 'open_terminal':
			is_done, msg = Server().open_terminal()
		elif command == 'close_terminal':
			is_done, msg = Server().close_terminal()
		elif command == 'exit':
			is_done, msg = Server().exit()
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
	("/src/(.*)",tornado.web.StaticFileHandler, {"path": "./src"},),
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
