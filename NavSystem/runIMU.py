#!/usr/bin/env python
import sys
from IMU.imu import * 
from madgwick_py.madgwickahrs import *
from scipy import signal
import smbus
import math
import numpy as np
import time


FLOAT_EPS_4 = np.finfo(float).eps * 4.0
one = np.array([[0, 0, 0], [0, 0, 0], [1, 1, 1]])

def quat2euler(q):
	K = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]], float)
	K[0, 0] = 2*q[0]**2-1+2*q[1]**2
	K[1, 0] = 2*(q[1]*q[2]-q[0]*q[3])
	K[2, 0] = 2*(q[1]*q[3]+q[0]*q[2])
	K[2, 1] = 2*(q[2]*q[3]-q[0]*q[1])
	K[2, 2] = 2*q[0]**2-1+2*q[3]**2
	phi = math.atan2(K[2,1], K[2,2])
	theta = -math.atan(K[2, 0]/math.sqrt(1-K[2, 0]**2))
	psi = math.atan2(K[1, 0], K[0, 0])
	return [phi, theta, psi]
def create_matrix(phi, theta,psi):
	R = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]], float)
	R[0,0] = (math.cos(psi) * math.cos(theta))
	R[0,1] = -math.sin(psi)*math.cos(phi) + math.cos(psi)*math.sin(theta)*math.sin(phi)
	R[0,2] = math.sin(psi)*math.sin(phi)+math.cos(psi)*math.sin(theta)*math.cos(phi)
	R[1,0] = math.sin(psi)*math.cos(theta)
	R[1,1] = math.cos(psi)*math.cos(phi) + math.sin(psi)*math.sin(theta)*math.sin(phi)
	R[1,2] = -math.cos(psi)*math.sin(phi) + math.sin(psi)*math.sin(theta)*math.cos(phi)
	R[2,0] = -math.sin(theta)
	R[2,1] = math.cos(theta)*math.sin(phi)
	R[2,2] = math.cos(theta)*math.cos(phi)
	return R

bus = smbus.SMBus(1)
imu_controller = IMU(bus, 0x69, 0x53, 0x1e, "IMU")
imu_controller.set_compass_offsets(9, -10, -140)
maj = MadgwickAHRS(0.014)
linvel = 0
linvel_last = 0
linpos_last = 0
linpos = 0
if __name__ == "__main__":
	while True:
		last_time = time.time()
		accel =  np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]], float)
		for i in range(0, 3):			
			(pitch,roll, gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x,accel_scaled_y, accel_scaled_z) = imu_controller.read_all()
#			print "gyrox {:f} , gyro y {:f}, gyro z {:f}, accelx{:f}, accely{:f}, accelz{:f}".format(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x,accel_scaled_y, accel_scaled_z)
			accel[i] = [accel_scaled_x,accel_scaled_y,accel_scaled_z]
			if i == 0:
				maj.update_imu([gyro_scaled_x, gyro_scaled_y, gyro_scaled_z], [accel_scaled_x,accel_scaled_y, accel_scaled_z])
		YPR = quat2euler(maj.quaternion)
		rotate_matrix = create_matrix(YPR[0], YPR[1], YPR[2])
		rotate_matrix = rotate_matrix.transpose()
		accel = accel.transpose()
		tcAcc = np.dot(rotate_matrix, accel)

		tcAcc[2,2] = tcAcc[2,2]-0.04
		tcAcc[2,1] = tcAcc[2,1]-0.04
		tcAcc[2,0] = tcAcc[2,0]-0.04

		linAcc = (tcAcc-one)*9.81
		now = time.time()
		time_deff = now - last_time
		linvel = linvel_last + linAcc*time_deff
		linvel = linvel.transpose()
		b, a = signal.butter(1, 0.2/62.5, "highpass")
		linVelHP = signal.filtfilt(b, a, np.ravel(linvel))
		linpos = linpos_last+linVelHP*time_deff
		linposHP = signal.filtfilt(b,a,np.ravel(linVelHP))
#		print "X {:f} Y {:f} Z {:f}".format(linposHP[0], linposHP[1], linposHP[2])
		print linVelHP
		last_time = now
		linvel_last = linvel
		linpos_last = linpos
		

