#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu, MagneticField
import math


def callback(data):

    theta =(180 * math.atan(data.magnetic_field.y / data.magnetic_field.x)) / math.pi
    print(theta)


def listener():
    rospy.init_node('mag_conv', anonymous=True)
    rospy.Subscriber("imu/mag", MagneticField, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
