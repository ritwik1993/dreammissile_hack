#!/usr/bin/env python

import time
import thread
import rospy
import roslib
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import sys


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


def servo_control():

	
	rospy.init_node('missile' , anonymous=True)
	rospy.Subscriber("track_point", Point, callback)
	#rospy.Subscriber("Fire_status", Bool, callback1)
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
	#if cmd != LAUNCHER_STOP:
	#	rospy.sleep(1)
	#	driverFile = open(LAUNCHER_NODE, 'w')
	#	driverFile.write(chr(LAUNCHER_STOP))
	#	driverFile.close()


def callback(data):
	#print "RECEIVED DATA"
	
	cmd = 0x00

	# PANNING IS BACKWARDS
	if data.x > 370:
		#print "RIGHT"
		cmd += LAUNCHER_RIGHT
	elif data.x < 270:
		#print "LEFT"
		cmd += LAUNCHER_LEFT

	if data.y > 290:
		#print "DOWN"
		cmd += LAUNCHER_DOWN
	elif data.y < 190:
		#print "UP"
		cmd += LAUNCHER_UP

	if cmd == 0x20:
		#print "STOP"
		cmd = LAUNCHER_STOP

	launcher_cmd(cmd)

def callback1(data1):
	print "RECEIVED FIRE DATA"
	
	#cmd = 0x00
	while data1 == True:
		print "Firing"	
		cmd = LAUNCHER_FIRE
		
		launcher_cmd(cmd)


def main(args):
	servo_control()
   

if __name__ == '__main__':
    main(sys.argv)
