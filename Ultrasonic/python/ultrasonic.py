import serial
import sys

class Ultrasonic(object):

	def __init__(self, port, baud=115200):
		self.ser = serial.Serial(port, baud)

	def read(self):
		self.ser.write('\x01')
		line = self.ser.read()
		sys.stdout.write(line)
		self.ser.flushInput()
