#-*- coding:utf-8 -*-
#from serial import serial
import serial
import numpy as np
import time 
import sys

import rospy

import os

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from std_msgs.msg import String
import json
import math

ser = None
Censor_Cof_A = np.array([[-2.171, 0.4, -0.137],[-0.095, -2.788, -0.456],[0.323, 0.172, 1.826]])
Censor_Cof_T = np.array([-0.215, 0.609, -0.913])

init_press_arr = np.zeros(3)


# raw_val : 2byte int * 4element
def calc_f_from_arr(raw_arr):
	global init_press_arr
	dv_arr = np.array([raw_arr[0], raw_arr[1], raw_arr[2], raw_arr[3]]) * 3.3 /1024.0
	dv_arr_dash = dv_arr[0:3] - Censor_Cof_T * dv_arr[3]
	return np.dot(Censor_Cof_A ,dv_arr_dash) - init_press_arr

# character からint　array に変換	
def convert_array_from_char(str):
	raw_arr = np.empty(4)
	for i in range(4):
		int_num = int(str[2+i*4 : 6+i*4], 16) # 2-5, 6-9, 10-15, 16-19 X, Y, Z, T
		raw_arr[i] = int_num
	return raw_arr		
	
def one_read(ser):
	ser.write("020201")
	raw_str = ser.read(20)
	print raw_str
	raw_arr = convert_array_from_char(raw_str)
	print raw_arr
	return calc_f_from_arr(raw_arr)
	
	

def init_serial():
	dev_list = os.listdir('/dev/')
	for i in range(5):
		dev_name = "ttyACM" + str(i)
		if dev_name in dev_list:
			break 
	print dev_name
	full_dev_path = "/dev/" + dev_name
	ser = serial.Serial(full_dev_path, timeout=0.1) 
	ser.write("020301")
	return ser

# センサ初期値 補正
def init_device(ser):
	global init_press_arr
	time.sleep(2.0) # wait for stable
	
	sum_arr = np.zeros(3)
	get_num = 10
	for i in range(get_num):
		time.sleep(0.2)
		sum_arr += one_read(ser)

	init_press_arr = sum_arr/get_num
	print "init_press arr = "
	print init_press_arr

def joy(pub, x,y, buttons = []):
	joy = joy()
	joy.axes = [x,y]
	joy.buttons = buttons
	pub.publish(joy)
	



def exit():
	global ser
	ser.close()
	print "successfully ended"
	
#main
if  __name__ == "__main__":

	#node_name = 'joy_twist'
	#rospy.init_node(node_name)
	#twist_pub = rospy.Publisher('iwm_twist', Twist, queue_size = 1)
	
	node_name = 'www_node'
	rospy.init_node(node_name)
	www_pub = rospy.Publisher('/chatter', String, queue_size = 1)


	ser = init_serial()
	init_device(ser)
	import atexit
	atexit.register(exit)

	
	while True:
		press_arr =  one_read(ser)
		print press_arr
		print "\n\n"

		x = press_arr[0]*40
		y = press_arr[1]*40
		z = press_arr[2]*30
		pos = {"x": x , "y" : y, "z" : z }

		json_str = json.dumps(pos)
		www_pub.publish(json_str)
	

		#twist = Twist()
		#twist.linear.x = press_arr[0]/10.0
		#twist.angular.z = press_arr[1]/10.0
		#twist_pub.publish(twist)
		
		time.sleep(0.1)
	
	ser.close()

