import serial

class Beacon(object):

	def __init__(self, ser, target=0x57, d_size=8):
		self.target = target
		self.d_size = 8
		self.ser = ser 

	def read(self):
		self.ser.write("\x01")
		ir_data = ord(self.ser.read() )
		self.ser.flush()
		return ir_data
