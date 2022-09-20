#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import math
from std_msgs.msg import Float64

pub = rospy.Publisher('control', Float64, queue_size = 1)

def callback(msg):

	kp = 0.04
	kd = -0.64

	theta = math.atan(((msg.linear_acceleration.x)/abs(msg.linear_acceleration.z))*180/math.pi)
	thetadot = msg.angular_velocity.y
	error = kd * ((kp/kd) * theta + thetadot)
	pub.publish(error)
	y = (msg.orientation.y)*180 / math.pi
	print (y)

def listener():

	rospy.init_node('control', anonymous=True)
	rospy.Subscriber("imu/data_raw", Imu, callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
