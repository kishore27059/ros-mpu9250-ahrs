#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import math
import numpy as np
import std_msgs.msg import Float64

pub = rospy.Publisher ('control', Float64, queue_size = 1)

def callback(msg):

    A = np.array([[1, dt], [0, 1]])
    C = np.array([[0,1], [1,0]])
    P = np.array([[1,2] , [2,3]])
    Q = 10.1 * np.array([[1,2] , [2,3]])
    R = 10.1 * np.array([[1,0], [0,1]])
    x = np.array([[0.0], [0.0]])
    theta = 0.0
    while True:

        theta = math.atan(((msg.linear_acceleration.x)/abs(msg.linear_acceleration.z))*180/math.pi)
        thetadot = msg.angular_velocity.x
        #theta = theta + dt
        #thetadot = 1 + ran.random()
        print(theta, thetadot)
        y = np.array([[theta], [thetadot]])

        #measurement update

        K = np.matmul(np.matmul(P,np.transpose(C)), np.linalg.inv((np.add((np.linalg.multi_dot([C, P, np.transpose])), R))))
        x += np.matmul(K, np.subtract(y, np.matmul(C, x)))
        P = np.matmul(np.subtract([[1,0], [0,1]],  np.matmul(K,C)) , P)

        #time update

        x = np.matmul(A,x)
        P = np.add(np.linalg.multi_dot([A, P, np.transpose(A)]), Q)
        print(x)
