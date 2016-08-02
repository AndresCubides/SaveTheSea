#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       ePuck.py
#       
#       Copyright 2010 Manuel Martín Ortiz <manuel.martin@itrblabs.eu>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 3 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#
#		-- ePuck.py --
#	
#		The aim of this library is to provide access to the ePuck robots
#		through a bluetooth connection. Thus, you can write a program that 
#		read from the ePuck's sensors and write in their actuators, This 
#		will allow us to create advanced programs that can develop a wide 
#		variety of complex tasks. It is necesary that the ePuck has installed 
#		the Webot's fimware 1.4.2 or 1.4.3. You can find this fantastic 
#		simulator on this site: http://www.cyberbotics.com/
#
#		This library is written in Python 2.6, and you can import it from
#		any program written in Python  (same version or later). In addition 
#		to this, you will also need two extra libraries:
#
#			-> Python Bluetooth or Pybluez
#			-> Python Image Library (PIL)
#
#		In this package you will find some examples of how to use this library.
#
#		You may expetience some problems when you work with your ePuck, We 
#		recommend you take into consideration the following special 
#		characteristic: we use a bluetooth communciation, therefore our bandwith 
#		is limited and we cannot expect to do too many tasks in short 
#		time; i.e:  If you put the wheels speed to max and want 
#		to make a quick process of the images, you will know what I'm saying. 
#		So remember, you are processing in your computer, not on the ePuck, 
#		and you need to take the sensors data and write on the actuators 
#		values on the ePuck
#
#		For further information and updates visit http://www.itrblabs.eu


from __future__ import division
import sys			# System library
import bluetooth	# Used for communications
import time			# Used for image capture process
import struct 		# Used for Big-Endian messages
import Image  		# Used for the pictures of the camera

from matplotlib import pyplot as pl
import numpy as np
import copy as cp

__package__ = "ePuck"
__docformat__ = "restructuredtext"  

"""
:newfield company: Company
"""

__version__ = "1.2.2"
__author__  = "Manuel Martin Ortiz"
__license__ =  "GPL"
__company__ = "RTDI - ITRB Labs"
__contact__ = ["manuel.martin@itrblabs.eu"]

# This dictionary have as keys the first character of the message, that
# is used to know the number of lines. If no key for the message, 1 line is assumed
DIC_MSG = {
	"v": 2,   	# Version
	"\n": 23, 	# Menu
	"\x0c": 2,  # Welcome
	"k": 3,   	# Calibration
	"R": 2	  	# Reset
}

# You have to use the keys of this dictionary for indicate on "enable" function 
# the sensor that you want to read 
DIC_SENSORS = { 
	"accelerometer" : "a",
	"selector" : "c",
	"motor_speed" : "e",
	"camera" : "i",
	"floor"  : "m",
	"proximity" : "n",
	"light" : "o",
	"motor_position" : "q",
	"microphone" : "u"
}
			
# You have to use the keys of this dictionary for indicate the operating 
# mode of the camera
CAM_MODE = {
	"GREY_SCALE"	: 0,
	"RGB_365"		: 1,
	"YUV"			: 2,
	"LINEAR_CAM"	: 3
	}

# You can use three diferents Zoom in the camera
CAM_ZOOM = (1, 4, 8)

class ePuck():
	"""
	This class represent an ePuck object
	"""
	
	def __init__(self, address, debug = False):
		# coors, orientation,
		"""
		Constructor process
		
		:param 	address: Robot's direction in AA:BB:CC:DD:EE:FF format
		:type	address: MAC Address
		:param 	debug: If you want more verbose information, useful for debugging
		:type	debug: Boolean

		:return: ePuck object
		"""
		# Monitoring Variables
		self.messages_sent = 0
		self.messages_received = 0
		self.version = __version__
		self.debug = debug
		#------------------------------Added Variables to the library code------------------------------------------------------------------------------------------------
		#Monitoring variables
		self.cleaning = False #Defines the status of the robot if it is cleaning or not
		self.trash_picked = False #True only when he arrive to a trash then its false again
		self.returning = False #True when the robot start coming back to its base until he arrive
		self.arrived = False #True only when he just arrive from a trash 
		self.cleaner_type = None #Saves the type of trash the robot can clean
		self.test = np.zeros((26,26)) #Only for plottin purposes in case of a mistake
		
		#Ubication Variables
		self.initialPos = np.zeros((2,1)) #Saves the robots base position
		self.coors = np.zeros((2,1)) #Saves the robots current position
		self.orientation = 0 #Saves the robots orientation

		# Moving Variables
		self.objective = None #Saves the coordinates of the objective the robot have to go
		self.angAnt = 0 #Saves the last correct angle of the robot to check if there is a difference too big with the actual to detect a mistake in the angle
		self.integral = 0 #PID integral value for the rotation
		self.last_proportional = 0 #PID last proportional value for rotation
		self.intDist = 0 #PID integral value for the longitudinal movement
		self.last_propDist = 0 #PID last proportional value for the longitudinal movement
		self.moving = False #True when the robot starts moving forward and not only rotating
		self.firstDist = 0 #First measured distance to the objective to checked if its decreasing or if its increasing (in the last case movement change to backwards)
		self.cnt = 0 #Count the number of loops since the robot started moving forward so he can check if he is going right or he have to go backwards
		self.backwards = False #True when the robot distance is increasing and it have to go backwards
		#-----------------------------------------------------------------------------------------------------------------------------------------------------------------

		# Connection Attributes
		self.socket = None
		self.address = address
		self.conexion_status = False
		
		# Camera attributes
		self._cam_width = None
		self._cam_height = None
		self._cam_enable = False
		self._cam_zoom = None
		self._cam_mode = None
		self._cam_size = None
		
		# Sensors and actuators lists
		self._sensors_to_read = []
		self._actuators_to_write = []
		
		# Sensors
		self._accelerometer = (0, 0, 0)
		self._accelerometer_filtered = False
		self._selector = (0)
		self._motor_speed = (0, 0) # left and right motor
		self._motor_position = (0, 0) # left and right motor
		self._camera_parameters = (0, 0, 0, 0)
		self._floor_sensors = (0, 0, 0)
		self._proximity = (0, 0, 0, 0, 0, 0, 0, 0)
		self._light_sensor = (0, 0, 0, 0, 0, 0, 0, 0)
		self._microphone = (0, 0, 0)
		self._pil_image = None
		
		# Leds
		self._leds_status = [False] * 8
		
	#This method was inserted to reset the robot variables when the game is restarting
	def reseting(self):
		#Monitoring Variables
		self.cleaning = False
		self.trash_picked = False
		self.returning = False
		self.arrived = False
		self.test = np.zeros((26,26))
		
		#Ubication Variables
		self.initialPos = np.zeros((2,1))
		self.coors = np.zeros((2,1))
		self.orientation = 0

		# Moving Variables
		self.objective = None
		self.angAnt = 0
		self.integral = 0
		self.last_proportional = 0
		self.intDist = 0
		self.last_propDist = 0
		self.moving = False
		self.firstDist = 0
		self.cnt = 0
		self.backwards = False

	def _debug(self, *txt):
		"""
		Show debug information and data, only works if debug information
		is enable (see "set_debug()")
		
		:param 	txt: Data to be showed separated by comma
		:type	txt: Any
		"""
		##print 'prueba17'
		if self.debug:
			#print 'prueba18'
			print >> sys.stderr, '\033[31m[ePuck]:\033[0m ', ' '.join([str(e) for e in txt])
			#print 'prueba19'
		return 0
	
	def _recv(self, n = 4096):
		"""
		Receive data from the robot
		
		:param	n: 	Number of bytes you want to receive
		:type	n: 	int
		:return: 	Data received from the robot as string if it was successful, raise an exception if not
		:rtype:		String
		:raise Exception:	If there is a communication problem
		"""
		if not self.conexion_status:
			raise Exception, 'There is not connection'
		try:
			line = self.socket.recv(n)
			self.messages_received += 1
		except bluetooth.btcommon.BluetoothError, e:
			txt = 'Bluetooth communication problem: ' + str(e)
			self._debug(txt)
			raise Exception, txt
		else:
			return line
			
	def _send(self, message):
		"""
		Send data to the robot
		
		:param	message: Message to be sent
		:type	message: String
		:return: Number of bytes sent if it was successful. -1 if not
		:rtype:	int
		"""
		if not self.conexion_status:
			raise Exception, 'There is not connection'

		try:
			n = self.socket.send(message)
			self.messages_sent += 1
		except Exception, e:
			self._debug('Send problem:', e)
			return -1
		else:
			return n
		
	def _read_image(self):
		"""
		Returns an image obtained from the robot's camera. For communication
		issues you only can get 1 image per second
		
		:return: The image in PIL format
		:rtype: PIL Image
		"""	
		
		# Thanks to http://www.dailyenigma.org/e-puck-cam.shtml for
		# the code for get the image from the camera
		msg = struct.pack(">bb", - ord("I"), 0)
		
		try:
			n = self._send(msg)
			self._debug("Reading Image: sending " + repr(msg) + " and " + str(n) + " bytes")
				
			# We have to add 3 to the size, because with the image we
			# get "mode", "width" and "height"
			size = self._cam_size + 3
			img = self._recv(size)
			while len(img) != size:
				img += self._recv(size)	
				
			# Create the PIL Image
			image = Image.frombuffer("RGB", (self._cam_width, self._cam_height), 
									 img, "raw", 
									 "BGR;16", 0, 1)
									 
			image = image.rotate(180)
			self._pil_image = image
			
		except Exception, e:
			self._debug('Problem receiving an image: ', e)

	def _refresh_camera_parameters(self):
		"""
		Method for refresh the camera parameters, it's called for some
		private methods
		"""
		try:
			msg = self.send_and_receive("I").split(',')
		except:
			return False
		else:
			self._cam_mode, \
			self._cam_width, \
			self._cam_height, \
			self._cam_zoom, \
			self._cam_size = [int(i) for i in msg[1:6]]
			
			self._camera_parameters = self._cam_mode, self._cam_width, self._cam_height, self._cam_zoom
						
	def _write_actuators(self):
		"""
		Write in the robot the actuators values. Don't use directly,
		instead use 'step()'
		"""

		# Not all messages reply with AKC, only Ascii messages
		acks = ['j', 't']
		# We make a copy of the actuators list
		actuators = self._actuators_to_write[:]
		for m in actuators:
			if m[0] == 'L':
				# Leds
				msg = struct.pack('<bbb', - ord(m[0]), m[1], m[2])
				n = self._send(msg)
				self._debug('Binary message sent of [' + str(n) + '] bytes: ' + str(struct.unpack('<bbb', msg)))
				
			elif m[0] == 'D' or m[0] == 'P':
				# Set motor speed or set motor position
				msg = struct.pack('<bhh', - ord(m[0]), m[1], m[2])
				n = self._send(msg)
				self._debug('Binary message sent of [' + str(n) + '] bytes: ' + str(struct.unpack('<bhh', msg)))
				
			else:
				# Others actuators, parameters are separated by commas
				msg = ",".join(["%s" % i for i in m])
				reply = self.send_and_receive(msg)
				if reply == 'j':
					self._refresh_camera_parameters()
				if reply not in acks:
					self._debug('Unknown ACK reply from ePcuk: ' + reply)
			self._actuators_to_write.remove(m)
		return
			
	def _read_sensors(self):
		"""
		This method is used for read the ePuck's sensors. Don't use directly,
		instead use 'step()'
		"""
		
		# We can read sensors in two ways: Binary Mode and Ascii Mode
		# Ascii mode is slower than Binary mode, therefore, we use
		# Binary mode whenever we can. Not all sensors are available in
		# Binary mode
		def send_binary_mode(parameters):
			# Auxiliar function for sent messages in binary modes
			# Parameters: ('Char to be sent', 'Size of reply waited', 'Format of the teply')
			self._debug('Sending binary message: ', ','.join('%s' % i for i in parameters))
			message = struct.pack(">bb", - ord(parameters[0]), 0)
			self._send(message)
			reply = self._recv()
			while len(reply) < parameters[1]:
				reply += self._recv()
			reply = struct.unpack(parameters[2], reply)
			
			self._debug('Binary message recived: ', reply)
			return reply
				
		# Read differents sensors
		for s in self._sensors_to_read:

			if s == 'a':
				# Accelerometer sensor in a non filtered way
				if self._accelerometer_filtered:
					parameters = ('A', 12, '@III')
					
				else:
					parameters = ('a', 6, '@HHH')
				
				reply = send_binary_mode(parameters)
				if type(reply) is tuple and type(reply[0]) is int:
					self._accelerometer = reply
				
			elif s == 'n':
				# Proximity sensors
				parameters = ('N', 16, '@HHHHHHHH')
				reply = send_binary_mode(parameters)
				if type(reply) is tuple and type(reply[0]) is int:
					self._proximity = reply
			
			elif s == 'm':
				# Floor sensors
				parameters = ('M', 10, '@HHHHH')
				reply = send_binary_mode(parameters)
				if type(reply) is tuple and type(reply[0]) is int:
					self._floor_sensors = reply
					
			elif s == 'q':
				# Motor position sensor
				parameters = ('Q', 4, '@HH')
				reply = send_binary_mode(parameters)
				if type(reply) is tuple and type(reply[0]) is int:
					self._motor_position = reply
					
			elif s == 'o':
				# Light sensors
				parameters = ('O', 16, '@HHHHHHHH')
				reply = send_binary_mode(parameters)
				if type(reply) is tuple and type(reply[0]) is int:
					self._light_sensor = reply
					
			elif s == 'u':
				# Microphone
				parameters = ('u', 6, '@HHH')
				reply = send_binary_mode(parameters)
				if type(reply) is tuple and type(reply[0]) is int:
					self._microphone = reply
					
			elif s == 'e':
				# Motor Speed
				parameters = ('E', 4, '@HH')
				reply = send_binary_mode(parameters)
				if type(reply) is tuple and type(reply[0]) is int:
					self._motor_speed = reply
				
			elif s == 'i':
				# Do nothing for the camera, is an independent process
				pass
				
			else:
				reply = self.send_and_receive(s).split(",")
			
				t = reply[0]
				response = tuple(reply[1:len(reply)])
					
				if t == "c": 
					# Selector
					self._selector = response[0]
					
				else:
					self._debug('Unknow type of sensor to read' + str(reply))
	
	
	#
	# Public methods
	#
	
	#The connecting method was modified
	def connect(self):
		"""
		Connect with the physic ePuck robot
		
		:return: If the connexion was succesful
		:rtype: Boolean
		:except Exception: If there are a communication proble, for example, the robot is off
		"""
		
		if self.conexion_status:
			self._debug('Already connected')
			return False

		n = 0 #Variable that saves how many times the connection have been attempted

		while (n != 10) & (n != 11): #Checks if the connection is not already correct or the maximum attempts have been reached
			try:
				self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
				self.socket.settimeout(3)
				self.socket.connect((self.address, 1))
				n = 11 #If the connection is succesful there is no need to continue with the while
				
			except Exception, e:
				print 'Try again'
				n += 1 #If the connection failed the attempt counter increase

		if n == 10: #Checks if the maximum number of attempts have been reached
			txt = 'Connection problem: \n' + str(e)
			self._debug(txt)
			raise Exception, txt
		
		self.conexion_status = True
		self._debug("Connected")

		self.reset()
		return True

	# def connect(self):
	# 	"""
	# 	Connect with the physic ePuck robot
		
	# 	:return: If the connexion was succesful
	# 	:rtype: Boolean
	# 	:except Exception: If there are a communication proble, for example, the robot is off
	# 	"""
		
	# 	if self.conexion_status:
	# 		self._debug('Already connected')
	# 		return False
	# 	try:
	# 		self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
	# 		self.socket.connect((self.address, 1))
	# 		self.socket.settimeout(5)
			
	# 	except Exception, e:
	# 		txt = 'Connection problem: \n' + str(e)
	# 		self._debug(txt)
	# 		raise Exception, txt
		
	# 	self.conexion_status = True
	# 	self._debug("Connected")

	# 	self.reset()
	# 	return True
		
	def disconnect(self):
		"""
		Disconnect from ePuck robot. Same as 'close()'
		"""
		
		self.close()
		
	def close(self):
		"""
		Close the connection with the robot. Same as 'disconnect()'
		
		:return: 0 if all ok
		:rtype: int
		:raise Exception: if it was a problem closing the connection
		"""
		
		if self.conexion_status:
			try:
				# Stop the robot
				self.stop()
				
				# Close the socket
				self.socket.close()
				self.conexion_status = False
			except Exception, e:
				raise Exception, 'Closing connection problem: \n' + str(e)
			else:
				return 0
		
	def set_debug(self, debug):
		"""
		Set / unset debug information
		:param debug: True or False, as you want or not Debug information
		:type debug: Boolean
		"""
		
		self.debug = debug
		
	def send_and_receive(self, msg):
		"""
		Send an Ascii message to the robot and return the reply. You can
		use it, but I don't recommend, use 'enable()', 'disable()'
		and 'step()' instead 
		
		:param msg: The message you want to send
		:type msg:	String
		:return: Response of the robot
		:rtype: String
		"""
		
		# Check the connection
		if not self.conexion_status:
			raise Exception, 'There is not connection'
			
		# Make sure the Message is a string
		message = str(msg)
		
		# Add carriage return if not
		if not message.endswith('\n'):
			message += '\n'
		
		# Check the lines of the waited reply
		if message[0] in DIC_MSG:
			lines = DIC_MSG[message[0]]
		else:
			lines = 1
		self._debug('Waited lines:', lines)
		
		# We make 5 tries before desist
		tries = 1
		while tries < 5:
			# Send the message
			bytes = self._send(message)
			self._debug('Message sent:', repr(message))
			self._debug('Bytes sent:', bytes)
			
			try:
				# Receive the reply. As we want to receive a line, we have to insist
				reply = ''
				while reply.count('\n') < lines:
					reply += self._recv()
					if message[0] == 'R':
						# For some reason that I don't understand, if you send a reset
						# command 'R', sometimes you recive 1 or 2 lines of 'z,Command not found\r\n'
						# Therefor I have to remove it from the expected message: The Hello message
						reply = reply.replace('z,Command not found\r\n','')
				self._debug('Message received: ', reply)
				return reply.replace('\r\n','')

			except Exception, e:
				tries += 1
				self._debug('Communication timeout, retrying')
		
		
		
	def save_image(self, name = 'ePuck.jpg'):
		"""
		Save image from ePuck's camera to disk
		
		:param name: Image name, ePuck.jpg as default
		:type name: String
		
		:return: Operation result
		:rtype:  Boolean
		"""
		
		if self._pil_image:
			return self._pil_image.save(name)
		else:
			return False

	#--------------------------------------Setters and Getters added--------------------------------

	def set_initialPos(self, initialPos):
		self.initialPos = initialPos

	def set_coors(self, coors):
		self.coors = coors

	def set_orientation(self, orientation):
		self.orientation = orientation

	def set_objective(self, objective):
		self.objective = objective

	def set_cleaning(self, cleaning):
		self.cleaning = cleaning

	def set_returning(self, returning):
		self.returning = returning

	def set_angAnt(self, angAnt):
		self.angAnt = angAnt

	def set_integral(self, integral):
		self.integral = integral

	def set_last_proportional(self, last_proportional):
		self.last_proportional = last_proportional

	def set_intDist(self, intDist):
		self.intDist = intDist

	def set_last_propDist(self, last_propDist):
		self.last_propDist = last_propDist

	def set_moving(self, moving):
		self.moving = moving

	def set_firstDist(self, firstDist):
		self.firstDist = firstDist

	def set_cnt(self, cnt):
		self.cnt = cnt

	def set_backwards(self, backwards):
		self.backwards = backwards

	def set_test(self, test):
		self.test = test

	def set_arrived(self, arrived):
		self.arrived = arrived

	def set_trash_picked(self, trash_picked):
		self.trash_picked = trash_picked

	def set_cleaner_type (self, cleaner_type):
		self.cleaner_type = cleaner_type
			
	def get_initialPos(self):
		return self.initialPos

	def get_coors(self):
		return self.coors

	def get_orientation(self):
		return self.orientation

	def get_objective(self):
		return self.objective

	def get_cleaning(self):
		return self.cleaning

	def get_returning(self):
		return self.returning

	def get_angAnt(self):
		return self.angAnt

	def get_integral(self):
		return self.integral

	def get_last_proportional(self):
		return self.last_proportional

	def get_intDist(self):
		return self.intDist

	def get_last_propDist(self):
		return self.last_propDist

	def get_moving(self):
		return self.moving

	def get_firstDist(self):
		return self.firstDist

	def get_cnt(self):
		return self.cnt

	def get_backwards(self):
		return self.backwards

	def get_test(self):
		return self.test

	def get_prueba(self):
		return self.prueba

	def get_trash_picked(self):
		return self.trash_picked

	def get_cleaner_type(self):
		return self.cleaner_type

	def get_arrived(self):
		return self.arrived
	#------------------------------------------------------------------------------------------------------------

	def get_accelerometer(self):
		"""
		Return Accelerometer values in (x, y, z)
		
		:return: Accelerometer values
		:rtype: Tuple
		"""
		return self._accelerometer
	
	def get_selector(self):
		"""
		Return the selector position (0-15)
		
		:return: Selector value
		:rtype: int
		"""
		return self._selector
	
	def get_motor_speed(self):
		"""
		Return the motor speed. Correct values are in the range [-1000, 1000]
		
		:return: Motor speed
		:rtype: Tuple
		"""
		return self._motor_speed
	
	def get_camera_parameters(self):
		"""
		Return the camera parameters as a tuple
		(mode, width, height, zoom)
		
		:return: Camera parameters
		:rtype: Tuple
		"""
		return self._camera_parameters
	
	def get_floor_sensors(self):
		"""
		Return the floor sensors values as (left, center, right)
		
		:return: Floor sensors values
		:rtype: Tuple
		"""
		return self._floor_sensors
	
	def get_proximity(self):
		"""
		Return the values of the 8 proximity sensors
		
		:return: Proximity sensors values
		:rtype: Tuple
		"""
		return self._proximity
	
	def get_light_sensor(self):
		"""
		Return the value of the light sensor
		
		:return: Ligth sensor value
		:rtype: Tuple
		"""
		return self._light_sensor
	
	def get_motor_position(self):
		"""
		Return the position of the left and right motor as a tuple
		
		:return: Motor position
		:rtype: Tuple
		"""
		return self._motor_position
	
	def get_microphone(self):
		"""
		Return the volume of the three microphones
		
		:return: Microphones values
		:rtype: Tuple
		"""
		return self._microphone
	
	def is_connected(self):
		"""
		Return a boolean value that indicate if the robot is connected to the PC
		
		:return: If the robot is connected to the PC
		:rtype: Boolean
		"""
		return self.conexion_status
		
	def get_image(self):
		"""
		Return the last image captured from the ePuck's camera (after a 'step()'). 
		None if	there are not images captured. The image is an PIL object
		
		:return: Image from robot's camera
		:rtype: PIL
		"""
		return self._pil_image
		
	def get_sercom_version(self):
		"""
		:return: Return the ePuck's firmware version
		:rtype: String
		"""
		return self.send_and_receive("v")
			
	def set_accelerometer_filtered(self, filter = False):
		"""
		Set filtered way for accelerometer, False is default value
		at the robot start
		
		:param filter: True or False, as you want
		:type filter: Boolean
		"""
		self._accelerometer_filtered = filter
		
	def disable(self, *sensors):
		"""
		Sensor(s) that you want to get disable in the ePuck
		
		:param sensors: Name of the sensors, take a look to DIC_SENSORS. Multiple sensors can be separated by commas
		:type sensors: String
		:return: Sensors enabled
		:rtype: List
		:except Exception: Some wrong happened
		"""
		for sensor in sensors:
			try:
				if not DIC_SENSORS.has_key(sensor):
					self._debug('Sensor "' + sensor + '" not in DIC_SENSORS')
					break
					
				if sensor == "camera":
					self._cam_enable = False
					
				if DIC_SENSORS[sensor] in self._sensors_to_read:
					l = list(self._sensors_to_read)
					l.remove(DIC_SENSORS[sensor])
					self._sensors_to_read = tuple(l)
					self._debug('Sensor "' + sensor + '" disabled')
				else:
					self._debug('Sensor "' + sensor + '" alrady disabled')
				
			except Exception, e:
				self._debug('Something wrong happened to disable the sensors: ', e)
		
		return self.get_sensors_enabled()
		
	def enable(self, *sensors):
		"""
		Sensor(s) that you want to get enable in the ePuck
		
		:param sensors: Name of the sensors, take a look to DIC_SENSORS. Multiple sensors can be separated by commas
		:type sensors: String
		:return: Sensors enabled
		:rtype: List
		:except Exception: Some wrong happened
		"""

		# Using the * as a parameters, we get a tuple with all sensors
		for sensor in sensors:
			try:
				if not DIC_SENSORS.has_key(sensor):
					self._debug('Sensor "' + sensor + '" not in DIC_SENSORS')
					break
					
				if sensor == "camera":
					# If the sensor is the Camera, then we refresh the 
					# camera parameters
					if not self._cam_enable:
						try:
							self._refresh_camera_parameters()
							self._cam_enable = True
							self.timestamp = time.time()
						except:
							break

				if DIC_SENSORS[sensor] not in self._sensors_to_read:
					l = list(self._sensors_to_read)
					l.append(DIC_SENSORS[sensor])
					self._sensors_to_read = tuple(l)
					self._debug('Sensor "' + sensor + '" enabled')
				else:
					self._debug('Sensor "' + sensor + '" already enabled')
				
			except Exception, e:
				self._debug('Something wrong happened to enable the sensors: ', e)
		return self.get_sensors_enabled()

	def get_sensors_enabled(self):
		"""
		:return: Return a list of sensors thar are active
		:rtype: List
		"""
		l = []
		for sensor in DIC_SENSORS:
			if DIC_SENSORS[sensor] in self._sensors_to_read:
				l.append(sensor)
		return l
				
	#Method to set the speed of each wheel of the robot, only one line added
	def set_motors_speed(self, l_motor, r_motor):
		"""
		Set the motors speed. The MAX and MIN speed of the ePcuk is [-1000, 1000]
		
		:param l_motor: Speed of left motor
		:type l_motor: int
		:param r_motor: Speed of right motor
		:type r_motor: int
		"""
		
		# I don't check the MAX and MIN speed because this check
		# will be made by the ePuck's firmware. Here we need speed
		# and we lose time mading recurrent chekings
		
		# self._actuators_to_write.append(("D", int(l_motor), int(r_motor)))
		
		# return True
		try:
			self._actuators_to_write.append(("D", int(l_motor), int(r_motor)))
			self.step() #This method needs to be called to make effective any change in the e-puck usually called outside the library

		except KeyboardInterrupt:
			print('\033', '[Log] ', 'Stoping the robot. Bye!')
			self.close()
			sys.exit()
		except Exception, e:
			print e
			
	def set_motor_position(self, l_wheel, r_wheel):
		"""
		Set the motor position, useful for odometry
		
		:param l_wheel: left wheel
		:type l_wheel: int
		:param r_wheel: right wheel
		:type r_wheel: int
		"""
		
		self._actuators_to_write.append(("P", l_wheel, r_wheel))
	
	#Method to turn on or off the leds, only one line added
	def set_led(self, led_number, led_value):
		"""
		Turn on/off the leds
		
		:param led_number: If led_number is other than 0-7, all leds are set to the indicated value.
		:type led_number: int
		:param led_value: 
			- 0 : Off
			- 1 : On (Red)
			- 2 : Inverse
		:type led_value: int
		"""
		
		led = abs(led_number)
		value = abs(led_value)
		
		if led < 9:
			self._actuators_to_write.append(("L", led, value))
			self.step() #This method needs to be called to make effective any change in the e-puck usually called outside the library
			if value == 0:
				self._leds_status[led] = False
			elif value == 1:
				self._leds_status[led] = True
			else:
				self._leds_status[led] = not self._leds_status[led]
			return True
		else:
			return False
	
	def set_body_led(self, led_value):
		"""
		Turn on /off the body led
		
		:param led_value: 
			- 0 : Off
			- 1 : On (green)
			- 2 : Inverse
		:type led_value: int
		"""
		
		value = abs(led_value)
		
		self._actuators_to_write.append(("L", 8, value))
		
		if value == 0:
				self._leds_status[8] = False
		elif value == 1:
			self._leds_status[8] = True
		else:
			self._leds_status[8] = not self._leds_status[8]
				
		return True
		
	def set_front_led(self, led_value):
		"""
		Turn on /off the front led
		
		:type	led_value: int
		:param 	led_value: 
			- 0 : Off
			- 1 : On (green)
			- 2 : Inverse
		"""
		value = abs(led_value)
		
		self._actuators_to_write.append(("L", 9, value))

		if value == 0:
				self._leds_status[9] = False
		elif value == 1:
			self._leds_status[9] = True
		else:
			self._leds_status[9] = not self._leds_status[9]
				
		return True
	
	def set_sound(self, sound):
		"""
		Reproduce a sound
		
		:param sound: Sound in the range [1,5]. Other for stop
		:type sound: int
		"""
		
		self._actuators_to_write.append(("T", sound))
		self.step()

		return True
					
	def set_camera_parameters(self, mode, width, height, zoom):
		"""
		Set the camera parameters
		
		:param mode: GREY_SCALE, LINEAR_CAM, RGB_365, YUM
		:type  mode: String
		:param width: Width of the camera
		:type  width: int
		:param height: Height of the camera
		:type  height: int
		:param zoom: 1, 4, 8
		:type  zoom: int
		"""
		
		if mode in CAM_MODE:
			self._cam_mode = CAM_MODE[mode]
		else:
			self._debug(ERR_CAM_PARAMETERS, "Camera mode")
			return -1
			
		if int(zoom) in CAM_ZOOM:
			self._cam_zoom = zoom
		else:
			self._debug(ERR_CAM_PARAMETERS, "Camera zoom")
			return -1
			
		if self.conexion_status and int(width) * int(height) <= 1600:
			# 1600 are for the resolution no greater than 40x40, I have
			# detect some problems
			self._actuators_to_write.append(("J", 
											 self._cam_mode, 
											 width,
											 height,
											 self._cam_zoom))
			return 0
				
	def calibrate_proximity_sensors(self):
		"""
		Calibrate proximity sensors, keep off any object in 10 cm
		
		:return: Successful operation
		:rtype: Boolean
		"""
		
		reply = self.send_and_receive("k",tries_timeout = 25)
		if reply[1] == "k":
			return True
		else:
			return False

	def reset(self):
		"""
		Reset the robot
		
		:return: Successful operation
		:rtype: Boolean
		:raise Exception: If there is not connection
		"""
		if not self.conexion_status:
			raise Exception, 'There is not connection'
			
		msg = self.send_and_receive("R")				
		self._debug(msg)
			
		return True
		
	def stop(self):
		"""
		Stop the motor and turn off all leds
		:return: Successful operation
		:rtype: Boolean
		:raise Exception: If there is not connection
		"""
		
		if not self.conexion_status:
			raise Exception, 'There is not connection'
			
		reply = self.send_and_receive("S")
		self._debug(reply)

		if reply == "s":
			return True
		else:
			return False
		
	def step(self):
		"""
		Method to update the sensor readings and to reflect changes in 
		the actuators. Before invoking this method is not guaranteed
		the consistency of the sensors
		"""

		if not self.conexion_status:
			raise Exception, 'There is not connection'	
			
		self._write_actuators()
		self._read_sensors()
		
		# Get an image in 1 FPS
		if self._cam_enable and time.time() - self.timestamp > 1:
			self._read_image()
			self.timestamp = time.time()

	#This method updates the PID output depending on it input (prop)
	def pid(self, prop, integ, last_prop, kp, ki, kd):
	    maxSp = 1000 #Maximum speed of the e-puck
	    derivative = np.absolute(prop) - np.absolute(last_prop) #Calculates the difference with the last proportional value to calculate the derivative factor
	    integ += prop #Integral factor that saves the acumulated error
	    last_prop = prop #Saves the new proportional value for next iteration
	    out = prop*kp + integ*ki + derivative*kd #Calculate the output of the PID
	    
	    #Checks if the output is greater than the maximum speed and sets the output to the maximum
	    if out > maxSp:
	        out = maxSp
	    if out < -maxSp:
	        out = -maxSp
	    return out, integ, last_prop #Return the speed, the integral value and the proportional so it can be saved for next iteration

	#Method that controls the movements of the e-puck
	def move(self, arr2d):

	    if (self.moving) & (35 < np.absolute(self.angAnt-self.orientation) < 90): #Checks if the angle has changed more than 35 in which
	    # case is a mistake in the robot orientation found, but if its over 90 its due to the change from negative to positive in X axis
	    	self.orientation = cp.copy(self.angAnt)

	    self.angAnt = cp.copy(self.orientation) #Saves the current angle to check with it next iteration

	    angObj = -1*np.rad2deg(np.arctan((self.objective[0]-self.coors[0])/(self.objective[1]-self.coors[1]))) #Calculates the angle between the robots position and the objective
	    #proportional = angObj - self.orientation #This substraction is if for any reason you don't want to flip the kinect image
	    proportional = self.orientation - angObj #Calculates the angle difference between the objective and the robot

	    if np.absolute(proportional) > 90: #If the angle difference is greater than 90 then that's not the shortest angle
	        if proportional < 0: #Checks if the angle is negative
	            proportional = proportional + 180 #Calculates the shortest angle in the opposite direction (sign change)
	        else:
	            proportional = proportional - 180 #Calculates the shortest angle in the opposite direction (sign change)

	    if (proportional >= -0.5) & (proportional <= 0.5):
	        self.moving = True #If the current angle difference is less than 0.5 then the robot starts moving forward

	    if self.moving: #Checks if the robot is moving forward
	        propDist = np.sqrt((self.objective[0] - self.coors[0])**2 + (self.objective[1] - self.coors[1])**2) #Calculates the distance between the objective and the robot
	        if self.firstDist == 0: #Checks if the first distance saved is zero then this is the first loop of the robot movement
	            self.firstDist = propDist #Saves the current distance as the initial one
	            #If the robot has moved the PID variable are reset
	            self.integral = 0 
	            self.last_proportional = 0

	        #Depende de la altura del kinect
	        if (propDist >= -2) & (propDist <= 2): #Checks if the robot current distance to the objective is less than 2 pixels (Be carefull this depends on the kinect height better to depend of the floor average depth)
	            self.set_motors_speed(0,0) #Stops the robot because he arrived to his objective
	            #Resets the PID values
	            self.integral = 0 
	            self.last_proportional = 0
	            self.intDist = 0
	            self.last_propDist = 0
	            #In case it was moving backwards it reset the variable
	            self.backwards = False
	            self.cnt = 0 #Reset the counting of loops per movement
	            self.firstDist = 0 #Resets the initial distance so next movement he saves the new distance to his new objective
	            self.moving = False #If he is going to move again he have to check first the angle so moving forward is set to False

	            distIni = np.sqrt((self.initialPos[0] - self.coors[0])**2 + (self.initialPos[1] - self.coors[1])**2) #Calculates the distance between the robot and his base

	            if (distIni >= -2) & (distIni <= 2): #Checks if the distance to the base is less than 2 px meaning he has arrive to his base and not a trash
	                self.angAnt = 0 #If the robot arrives to his base the angle is reseted 
	                self.objective = None #Objective is erased
	                self.cleaning = False #The robot is not cleaning anymore
	                self.returning = False #The robot already finished returning
	                self.arrived = True #The robot just arrived to his base
	                for i in range(8):
	                	self.set_led(i,0) #Turn off the leds of the robot
	            else: #If the distance to the base is not in the rank it means the robot arrived to the trash
	                self.returning = True #The robot is going to start returning to the base
	                self.trash_picked = True #The robot just tried to pick up a trash

	            print '!!!!!LLEGUE!!!!!!!!!!!'
	            return self #If the e-puck arrived to his objective or base it doesn't have to keep movin in the same direction so the method is stopped

	        #Calculates the longitudinal speed with the PID sending the distance between the robot and the objective as input   
	        speed, self.intDist, self.last_propDist = self.pid(propDist, self.intDist, self.last_propDist, 38, 1/1000, 5)

	        proportional = proportional*speed #The proportional value to the rotation speed while moving forward have to be proportional to the angle difference but 
	        #also to the longitudinal speed of the robot so it gets lower if the robot is stopping
	        
	        #Calculates the rotational speed with the PID sending the angle difference multiplied by the e-puck speed
	        rot_difference, self.integral, self.last_proportional = self.pid(proportional, self.integral, self.last_proportional, 1/48, 1/280000, 1/1000)

	        if np.absolute(rot_difference) >= np.absolute(speed): #Checks that the rotational speed is not bigger than the longitudinal speed so the robot won't turn mistakenly
	        	rot_difference = (rot_difference/np.absolute(rot_difference))*(np.absolute(speed)/3) #If the rotational speed is bigger than the longitudinal (probably due to 
	        	#an angle mistake not detected) then the rotational speed is assigne to a third of the longitudinal speed but with the same direction it had
	        	print 'speed MENOR!!!!!!!!!!!!!!!!!!!!'
	            
	            # #Uncomment to check why the rotation speed is greater that the longitudinal speed, some times is due to bad angle detection so thats why this was plotted
	            # self.set_motors_speed(0,0)
	            # pl.figure(figsize=(10,12))
	            # pl.subplot(1,2,1)
	            # pl.imshow(arr2d, vmin = 1700, vmax = 1900)
	            # pl.colorbar()
	            # pl.subplot(1,2,2)
	            # pl.imshow(self.test)
	            # pl.show()
	            # raw_input("Press Enter to terminate.")

	        if self.cnt == 4: #Checks if is the 4th loop after the e-puck started moving forward
	            if self.firstDist < propDist: #Checks if the distance between the objective and the robot has increased
	                print '!!!!!!!!!! BACKWARDS !!!!!!!!!!!!!!'
	                self.backwards = True #Sets the robot to move backwards

	        if self.backwards: #Checks if the robot is going backwards so the speed is set negative
	            if(rot_difference < 0): #Checks the direction of the rotation to see if is negative
	                self.set_motors_speed(-(speed + rot_difference), -speed) #Rotates to the robots right because the rotation speed is negative so its substracted from the left wheel. Sends left wheel speed and right wheel speed (in that order)
	            else:
	                self.set_motors_speed(-speed, -(speed - rot_difference)) #Rotates to the robots left because the rotation speed is positive so its substracted from the right wheel. Sends left wheel speed and right wheel speed (in that order)
	        else: #If the robot is not going backwards the speed is set positive
	            if(rot_difference < 0): #Checks the direction of the rotation to see if is negative
	                self.set_motors_speed(speed, speed + rot_difference) #Rotates to the robots right because the rotation speed is negative so its substracted from the right wheel. Sends left wheel speed and right wheel speed (in that order)
	            else:
	                self.set_motors_speed(speed - rot_difference, speed) #Rotates to the robots left because the rotation speed is positive so its substracted from the left wheel. Sends left wheel speed and right wheel speed (in that order)
	        self.cnt += 1 #Updates the number of loops since the robot started moving forward
	    else: #In case the robot is correcting just the orientation and not moving forward
	        #Calculates the rotational speed with the PID sending the angle difference as input   
	        rot_difference, self.integral, self.last_proportional = self.pid(proportional, self.integral, self.last_proportional, 7, 1/1100, 2)
	        self.set_motors_speed(-rot_difference, rot_difference) #Sets the robot wheel speeds to the PID output value but with opposite sign so it rotate. Sends left wheel speed and right wheel speed (in that order)

	    return self #Return the robot object with the changes made

