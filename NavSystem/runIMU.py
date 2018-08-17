#!/usr/bin/env python
"""
Данный код служит для позиционирования слепого человека в пространстве. Алгоритм работы следующий:
	1. Снимаются данные с датчиков ADXL345(акселлерометр) и HMC5883l(гироскоп)
	2. Снятые данные с датчиков обрабатываются алгоритмом Себастьяна Маджвика(спасибо ему огромное за проделанную работу!) который возвращает кватернион.
Алгоритм позволяет убрать дрейф гироскопа за счёт его совмещения с акселлерометром..
	3. Функция quat2euler преобразует кватернионы в углы Эйлера
	4. Функция create_matrix создаёт матрицу вращения по углам Эйлера
	5. Для получения tilt-compensated ускорения (ускорения без учитывания поворотов) матрица ускорения умножается на матрицу вращения
	6. Для получения линейного ускорения(без силы гравитации) из tilt-compensated матрицы вычитается 1G
	7. Линейное ускорение нормализуется(умножается на g = 9.81)
	8. Происходит интегрирование для получения скорости
	9. Для уменьшения дрейфа, скорость проходит через фильтр ВЧ Баттерворта
	10. Для получения позиции, скорость повторно интегрируется с шагом интегрирования, равным времени итерации полного кода
Код не является идеальным, поскольку при интегрировании получается большая ошыбка(пропорциональная квадрату времени). Для борьбы с этим 
нужно использовать более малошумящие и, следовательно, дорогие IMU-датчики.
"""

#импорт библиотек 
import sys
from IMU.imu import * 
from madgwick_py.madgwickahrs import *
from scipy import signal
import smbus
import math
import numpy as np
import time


one = np.array([[0, 0, 0], [0, 0, 0], [1, 1, 1]])#инициализация матрицы для вычитания силы гравитации с tilt-compensated ускорения

def quat2euler(q): # функция, конвентирующая кватернионы в углы Эйлера
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

def create_matrix(phi, theta,psi):#Функция, создающаяя матрицу вращения R
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
# инициализация IMU датчиков
bus = smbus.SMBus(1)
imu_controller = IMU(bus, 0x69, 0x53, 0x1e, "IMU")#Задаём i2c адреса

maj = MadgwickAHRS(0.014)#инициализируем ообъект фильтра Маджвика 
#переменные для интегрирования
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
			accel[i] = [accel_scaled_x,accel_scaled_y,accel_scaled_z]#создание матрицы 3x3 ускорений
			if i == 0:
				maj.update_imu([gyro_scaled_x, gyro_scaled_y, gyro_scaled_z], [accel_scaled_x,accel_scaled_y, accel_scaled_z])# получение кватернионов
		YPR = quat2euler(maj.quaternion)#получаем углы Эйлера
		rotate_matrix = create_matrix(YPR[0], YPR[1], YPR[2])
		rotate_matrix = rotate_matrix.transpose()#транспонирование матрицы вращения
		accel = accel.transpose()#транспонирование матрицы ускорения
		tcAcc = np.dot(rotate_matrix, accel)#умножение матрицы вращения на матрицу ускорения
		#поправочные коэфициенты
		tcAcc[2,2] = tcAcc[2,2]-0.04
		tcAcc[2,1] = tcAcc[2,1]-0.04
		tcAcc[2,0] = tcAcc[2,0]-0.04

		linAcc = (tcAcc-one)*9.81#нормализация ускорения
		now = time.time()
		time_deff = now - last_time
		linvel = linvel_last + linAcc*time_deff#интегрирование
		linvel = linvel.transpose()
		b, a = signal.butter(1, 0.2/62.5, "highpass")# ВЧ фильтр Баттерворта
		linVelHP = signal.filtfilt(b, a, np.ravel(linvel))
		linpos = linpos_last+linVelHP*time_deff# интегрирование скорости в позицию
		linposHP = signal.filtfilt(b,a,np.ravel(linVelHP))#ВЧ фильтр
		print "X {:f} Y {:f} Z {:f}".format(linposHP[0], linposHP[1], linposHP[2])
		#обновление данных для интегрирования
		last_time = now
		linvel_last = linvel
		linpos_last = linpos
		

