import numpy as np
import math

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

if __name__ == "__main__":

	k = create_matrix(	0.6381434/57.2958, 0.1694835/57.2958, 0.3897921/57.2958)
	acc = np.array([[-0.02880859, 0.01660156, 1.01709] ,[0.002441406,-0.002441406, 0.9736328], [0.006347656, 0.01660156, 0.9892578]], float)
	acc = acc.transpose()
	ss = np.dot(k, acc)
	ss = ss.transpose()
	linacc = (ss - one.transpose())*9.81

	print linacc

