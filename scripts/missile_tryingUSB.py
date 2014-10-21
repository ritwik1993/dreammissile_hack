#!/usr/bin/env python

import time
import thread
import rospy
import roslib
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import sys
#import usb
import usb.core
import usb.util


LAUNCHER_NODE = "/dev/launcher1"
LAUNCHER_FIRE = 0x10
LAUNCHER_STOP = 0x20
LAUNCHER_UP = 0x02
LAUNCHER_DOWN = 0x01
LAUNCHER_LEFT = 0x04
LAUNCHER_RIGHT = 0x08
#LAUNCHER_UP_LEFT = LAUNCHER_UP | LAUNCHER_LEFT
#LAUNCHER_DOWN_LEFT = LAUNCHER_DOWN | LAUNCHER_LEFT
#LAUNCHER_UP_RIGHT = LAUNCHER_UP | LAUNCHER_RIGHT
#LAUNCHER_DOWN_RIGHT = LAUNCHER_DOWN | LAUNCHER_RIGHT

class Turret:

	def __init__(self):

		self.launcher = Launcher()


	def servo_control(self):

		rospy.init_node('missile' , anonymous=True)
		rospy.Subscriber("track_point", Point, self.callback)
	#	orientation_pub = rospy.Publisher("orientation", JointState)
		rospy.spin()


	def getch(self):
		import sys, tty, termios
		fd=sys.stdin.fileno()
		old_settings=termios.tcgetattr(fd)
		try:
			tty.setraw(sys.stdin.fileno())
			ch=sys.stdin.read(1)
		finally:
			termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
		return ch


	def launcher_cmd(self, cmd):
		'''
		driverFile = open(LAUNCHER_NODE, 'w')
		driverFile.write(chr(cmd))
		driverFile.close()
		'''
		REQUEST_TYPE_SEND = usb.util.build_request_type(usb.util.CTRL_OUT, usb.util.CTRL_TYPE_CLASS, usb.util.CTRL_RECIPIENT_DEVICE)


	def callback(self, data):
		print "RECEIVED DATA"
	
		cmd = 0x00

		# PANNING IS BACKWARDS
		if data.x > 370:
			print "RIGHT"
			cmd += LAUNCHER_RIGHT
		elif data.x < 270:
			print "LEFT"
			cmd += LAUNCHER_LEFT

		if data.y > 290:
			print "DOWN"
			cmd += LAUNCHER_DOWN
		elif data.y < 190:
			print "UP"
			cmd += LAUNCHER_UP

		if cmd == 0x00:
			print "STOP"
			cmd = LAUNCHER_STOP

		self.launcher_cmd(cmd)

		self.launcher.check_limits()
	

class Launcher():
	def __init__(self):
		self.val = 0

		self.dev = usb.core.find(idVendor=0x2123, idProduct=0x1010)
		if self.dev is not None:
			dev_manufacturer = usb.util.get_string(self.dev, 256, 1)
			dev_name = usb.util.get_string(self.dev, 256, 1)
			print "Device found: " + dev_manufacturer + " " + dev_name

			interface = 0
			if self.dev.is_kernel_driver_active(interface) is True:
				print "We need to detach kernel driver"
				self.dev.detach_kernel_driver(interface)
				self.dev.set_configuration()
				print "Claiming device"
				usb.util.claim_interface(self.dev, interface)
				print "All ours"
			else:
				usb.util.claim_interface(self.dev, interface)
				print "Claimed it"


	def acquire(self, dev):
		self.handle = dev.open()

		try:
			self.handle.reset()
		except usb.USBError, e:
			if str(e).find("not permitted") >= 0:
				return 2
			else:
				raise e

		try:
			self.handle.claimInterface(0)
		except usb.USBError, e:
			if str(e).find("could not claim interface") >= 0:
				self.handle.detachKernelDriver(0)
				self.handle.claimInterface(0)

		self.handle.setAltInterface(0)

		return 0
	
					

	def check_limits(self):
		
		'''
		bytes = self.handle.bulkRead(1, 8)

		print "USB packet:", bytes

		limit_bytes = list(bytes)[0:2]
		self.previous_fire_state = limit_bytes[1] & (1 << 7)

		limit_signal = (limit_bytes[1] & 0x0F) | (limit_bytes[0] >> 6)

		new_limit_switch_states = [bool(limit_signal & (1 << i)) for i in range(4)]
		self.previous_limit_switch_states = new_limit_switch_states

		return new_limit_switch_states
		'''
		endpoint = self.dev[0][(0,0)][0]
		attempts = 10
		data = None

		'''
		while data is None and attempts > 0:
			try:
				data = self.dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize)
			except usb.core.USBError as e:
				data = None
				if e.args == ("Operation timed out"):
					attempts -= 1
					print "timed out ... trying again"
					continue
		'''
		data = self.dev.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize)

		print data


def main(args):
	turret = Turret()
	turret.servo_control()
   

if __name__ == '__main__':
    main(sys.argv)
