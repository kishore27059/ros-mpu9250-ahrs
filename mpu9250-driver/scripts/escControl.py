#!/usr/bin/env python3

import rospy
import os
import time
os.system ("sudo pigpiod")
time.sleep(1)
import pigpio
import math
from std_msgs.msg import Float64
import numpy as np

ESC1 = 4
ESC2 = 17

minValue = 1000
maxValue = 2000

pi = pigpio.pi()
pi.set_servo_pulsewidth (ESC1, 0)
pi.set_servo_pulsewidth (ESC2, 0)

time.sleep(1)

pi.set_servo_pulsewidth (ESC1, maxValue)
pi.set_servo_pulsewidth (ESC2, maxValue)

time.sleep(1)

pi.set_servo_pulsewidth (ESC1, minValue)
pi.set_servo_pulsewidth (ESC2, minValue)

time.sleep(1)


pub = rospy.Publisher('MotorSpeed', Float64 , queue_size = 2)

def callback(error):

	throttle = 0.75

	pwm1 = throttle - (error.data/ 2)
	pwm2 = throttle + (error.data/ 2)

	if pwm1 > 1:
		pwm1 = 1
	if pwm2 > 1:
		pwm2 = 1
	if pwm1 < 0.5:
		pwm1 = 0.5
	if pwm2 < 0.5:
		pwm2 = 0.5

	speed1 = 400 + (((pwm1 - 0.5)/0.5)*1000)
	speed2 = 400 + (((pwm2 - 0.5)/0.5)*1000)

#	speed = np.array([speed1, speed2])

#	pub.publish (speed)
#	pub.publish (speed2)

#	print ('Speed 1: ', speed1)
#	print ('Speed 2: ', speed2)

	pi.set_servo_pulsewidth(ESC1, speed1)
	pi.set_servo_pulsewidth(ESC2, speed2)

#	if input() == "stop":
#		pi.set_servo_pulsewidth(ESC1, 0)
#		pi.set_servo_pulsewidth(ESC2, 0)
#		pi.stop()
#		break
	time.sleep(0.1)

def listener():

	rospy.init_node('MotorSpeed', anonymous = True)
	rospy.Subscriber('error/calc', Float64 , callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
