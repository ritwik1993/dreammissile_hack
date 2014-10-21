#!/usr/bin/env python

import time
import thread
import rospy
import roslib
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import sys
import usb


LAUNCHER_NODE = "/dev/launcher0;"
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




def servo_control():

	rospy.init_node('missile' , anonymous=True)
	rospy.Subscriber("rocket_command", String, callback)
#	orientation_pub = rospy.Publisher("orientation", JointState)
	rospy.spin()


def getch():
	import sys, tty, termios
	fd=sys.stdin.fileno()
	old_settings=termios.tcgetattr(fd)
	try:
		tty.setraw(sys.stdin.fileno())
		ch=sys.stdin.read(1)
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	return ch


def launcher_cmd(cmd):
	driverFile = open(LAUNCHER_NODE, 'w')
	driverFile.write(chr(cmd))
	driverFile.close()


def callback(data):
	print "RECEIVED DATA"
	print data

	cmd = 0x00

	# PANNING IS BACKWARDS
	command_string = data.data.lower()
	if command_string == 'right':
		cmd = LAUNCHER_RIGHT
	elif command_string == 'left':
		cmd = LAUNCHER_LEFT
	elif command_string == 'down':
		cmd = LAUNCHER_DOWN
	elif command_string == 'up':
		cmd = LAUNCHER_UP
	elif command_string == 'fire':
		cmd = LAUNCHER_FIRE
	elif command_string == 'stop':
		cmd = LAUNCHER_STOP
	else:
		print "Invalid command string"
		cmd = LAUNCHER_STOP

	launcher_cmd(cmd)
	

	


def main(args):
	servo_control()
   

if __name__ == '__main__':
    main(sys.argv)
