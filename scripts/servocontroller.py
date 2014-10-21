#!/usr/bin/env python
import serial
import time
import thread
import rospy
import roslib
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import sys

ser = serial.Serial('/dev/ttyACM0', 9600)
cmdinitial=chr(0xFF)+chr(0x00)+chr(0x7F)
ser.write(cmdinitial)
cmdinitial1=chr(0xFF)+chr(0x01)+chr(0x7F)
ser.write(cmdinitial1)
orientation_pub = rospy.Publisher("joint_states", JointState)

def servo_control():

	rospy.init_node('servo_control' , anonymous=True)
	rospy.Subscriber("track_point", Point, callback)
	
	rospy.spin()
	ser.close()


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


def linear_interp(x0, y0, x1, y1, x_want):
	return y0 + (y1-y0) * ((x_want - x0) / (x1 - x0))

def get_new_JointState(pan, tilt):
	j = JointState()
	j.header.stamp = rospy.Time.now()
	j.name = ["panservo", "tiltservo"]
	j.position = [pan, tilt]
	j.velocity = []
	j.effort = []
	return j

tilt = 127.0
pan = 127.0
cmdpan = ""
cmdtilt =""
increment = 51.0


def callback(data):
	global tilt, pan, cmdpan, cmdtilt, increment
	#c=getch()
	#print data

	k = 0.07

	panincrement = float(data.x) - 320.0
	panincrement *= k

	tiltincrement = float(data.y) - 240.0
	tiltincrement *= k

	tilt -= tiltincrement
	if tilt > 254.0:
		tilt = 254.0
	elif tilt < 0.0:
		tilt = 0.0

	cmdtilt=chr(0xFF)+chr(0x00)+chr(int(tilt))


	# Pan needs to be added because the incoming image is backwards.  Therefore
	# the origin of the image is in the upper right rather than upper left.
	pan += panincrement
	if pan > 254.0:
		pan = 254.0
	elif pan < 0.0:
		pan = 0.0

	cmdpan=chr(0xFF)+chr(0x01)+chr(int(pan))


	ser.write(cmdtilt)
	ser.write(cmdpan)

	# Now publish our angles
	pan_rad = linear_interp( 254.0, -math.pi / 4.0, 0.0, math.pi / 4.0, pan )
	tilt_rad = linear_interp( 0.0, 0.0, 254.0, math.pi / 2.0, tilt )	

	j = get_new_JointState( pan_rad, tilt_rad )
	orientation_pub.publish( j )
	

def main(args):
	servo_control()
   
if __name__ == '__main__':
    main(sys.argv)
