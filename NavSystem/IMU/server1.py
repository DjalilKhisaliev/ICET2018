#!/usr/bin/env python

import smbus
import math
import imu as imu
import numpy as np
import time


FLOAT_EPS_4 = np.finfo(float).eps * 4.0
one = np.array([[0, 0, 0], [0, 0, 0], [1, 1, 1]])


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
imu_controller = imu.IMU(bus, 0x69, 0x53, 0x1e, "IMU")
imu_controller.set_compass_offsets(9, -10, -140)

linvel = 0
linvel_last = 0
if __name__ == "__main__":
	while True:
		last_time = time.time()
		accel =  np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]], float)
		for i in range(0, 3):			
			(pitch,roll, gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x,accel_scaled_y, accel_scaled_z) = imu_controller.read_all()
#			print "gyrox {:f} , gyro y {:f}, gyro z {:f}, accelx{:f}, accely{:f}, accelz{:f}".format(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x,accel_scaled_y, accel_scaled_z)
			accel[i] = [accel_scaled_x,accel_scaled_y,accel_scaled_z]
			if i == 0:
				(yaw, pitch, roll) = imu_controller.read_yaw_pitch_roll()
		rotate_matrix = create_matrix(yaw, pitch, roll)
#		print rotate_matrix
		accel = accel.transpose()
		#print accel
		tcAcc = np.dot(rotate_matrix, accel)	
#		print tcAcc
		linAcc = tcAcc-one
		now = time.time()
		time_deff = now - last_time
		linvel  =linvel_last + linAcc*time_deff	
		#print tcAcc
#		print accel[1]
#		print accel[2]
		#print "Yaw: {:f} Pitch: {:f} roll: {:f}".format(yaw, pitch, roll)
		print time_deff 
		last_time = now
		linvel_last = linvel
	
		

