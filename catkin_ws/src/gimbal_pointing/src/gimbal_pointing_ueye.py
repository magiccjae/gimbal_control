#!/usr/bin/env python
# Module to compare the two numbers and identify and error between sending via float and ASCII
import serial as sr
import numpy as np
import math as m
import rospy
from geometry_msgs.msg import Vector3
import time
import struct
import sys

f = 2900
eps_x = 0
eps_y = 0

def callback(data):
	global eps_x
	eps_x = data.x
	global eps_y
	eps_y = data.y
	if eps_x==-800:
		eps_x=0
	if eps_y==-600:
		eps_y=0

rospy.init_node('gimbal_pointing', anonymous=True)
rospy.Subscriber("pixel_location", Vector3, callback)

R_c_g = np.matrix([[0,0,1],[1,0,0],[0,1,0]])

def Rot_g_to_b(az,el):
	Rot_g1_to_b = np.matrix([[m.cos(az),-m.sin(az),0],[m.sin(az),m.cos(az),0],[0,0,1]])
	Rot_g_to_g1 = np.matrix([[m.cos(el),0,m.sin(el)],[0,1,0],[-m.sin(el), 0, m.cos(el)]])
	return Rot_g1_to_b*Rot_g_to_g1

def calculate_commanded_angles(az, el, eps_x, eps_y):
	ell_c = 1/m.sqrt(m.pow(f,2)+m.pow(eps_x,2)+m.pow(eps_y,2))*np.matrix([[eps_x],[eps_y],[f]])
	ell_b = Rot_g_to_b(az,el)*R_c_g*ell_c
	az_d = m.atan2(ell_b.item((1,0)),ell_b.item((0,0)))
	el_d = -m.asin(ell_b.item((2,0)))
	return (az_d, el_d)

print "gimbal_pointing_ueye started"
arduino = sr.Serial('/dev/ttyUSB0')		# dummy serial port to get rid of garbage values left over
with arduino:
	arduino.setDTR(False)
	time.sleep(1)
	arduino.flushInput()
	arduino.setDTR(True)

ser = sr.Serial('/dev/ttyUSB0', 57600)

while True:
	try:
		pitch_IMU = float(ser.readline())
		yaw_IMU = float(ser.readline())		
		pitch_encoder = float(ser.readline())
		yaw_encoder = float(ser.readline())
		print "IMU el, az", pitch_IMU, yaw_IMU		
		print "encoder el, az", pitch_encoder, yaw_encoder
		print "pixel_location", eps_x, eps_y
		(az_d,el_d) = calculate_commanded_angles(yaw_encoder*m.pi/180,pitch_encoder*m.pi/180,eps_x,eps_y)
		desired_el = round(el_d*180/m.pi, 2)
		desired_az = round(az_d*180/m.pi, 2)
		print "desired el, az", desired_el, desired_az
		bin = struct.pack('f', desired_az)
		for b in bin:		# float 4 bytes
			ser.write(b)
		bin2 = struct.pack('f', desired_el)
		for b in bin2:		# float 4 bytes
			ser.write(b)
		drift_az = float(ser.readline())
		drift_el = float(ser.readline())
		temp_az = float(ser.readline())
		temp_el = float(ser.readline())
		print "drift el, az", drift_el, drift_az
		print "commanded el, az", temp_el, temp_az
		time.sleep(0.2)	
	except KeyboardInterrupt:
		print "exiting...."
		ser.flushInput()
		ser.flushOutput()
		ser.close()
		print "serial port closed"
