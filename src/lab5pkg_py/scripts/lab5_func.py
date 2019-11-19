#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab5_header import *


"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for w1~6 and q1~6, as well as the M matrix
	#M = np.eye(4)
	x_offset = -0.149
	y_offset = 0.149
	z_offset = 0.0193

	M = np.array([[0,-1,0,0.54 + x_offset], [0,0,-1,0.242 + y_offset], [1,0,0,0.212 + z_offset], [0,0,0,1]])
	S = np.zeros((6,6))

	w1 = np.array([0,0,1])
	w2 = np.array([0,1,0])
	w3 = np.array([0,1,0])
	w4 = np.array([0,1,0])
	w5 = np.array([1,0,0])
	w6 = np.array([0,1,0])

	q1 = np.array([0 + x_offset			,0 + y_offset,		z_offset + 0])
	q2 = np.array([0 + x_offset			,0 + y_offset,		z_offset + 0.152])
	q3 = np.array([0.244 + x_offset		,0 + y_offset,		z_offset + 0.152])
	q4 = np.array([0.457 + x_offset		,0 + y_offset,		z_offset + 0.152])
	q5 = np.array([0 + x_offset			,0.11 + y_offset,	z_offset + 0.152])
	q6 = np.array([0.54 + x_offset		,0 + y_offset,		z_offset + 0.152])

	v1 = np.cross(-w1, q1)
	v2 = np.cross(-w2, q2)
	v3 = np.cross(-w3, q3)
	v4 = np.cross(-w4, q4)
	v5 = np.cross(-w5, q5)
	v6 = np.cross(-w6, q6)

	S[0][0] = w1[0] 
	for i in range(6):
		if(i<=2):
			S[0][i] = w1[i]
			S[1][i] = w2[i]
			S[2][i] = w3[i]
			S[3][i] = w4[i]
			S[4][i] = w5[i]
			S[5][i] = w6[i]
		else:
			S[0][i] = v1[i - 3]
			S[1][i] = v2[i - 3]
			S[2][i] = v3[i - 3]
			S[3][i] = v4[i - 3]
			S[4][i] = v5[i - 3]
			S[5][i] = v6[i - 3]

	
	# ==============================================================#
	return M, S

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Forward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	exp = []
	for i in range(6):
		'''
		print(i)
		print(S[i])
		print(theta[i])
		'''
		exp.append(get_exp(S[i], theta[i]))

	for i in range(6):
		T = T.dot(exp[i])
	T = T.dot(M)
	


	# ==============================================================#
	
	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value



def get_exp(S, theta):
	w = [S[0], S[1], S[2]]
	v = [S[3], S[4], S[5]]
	s_bracket = np.array([[0, -w[2] , w[1], v[0]], [w[2], 0, -w[0], v[1]], [-w[1], w[0], 0, v[2]], [0,0,0,0]])
	s_bracket_theta = s_bracket*theta
	return expm(s_bracket_theta)


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xwg, ywg, zwg, yaw):

 	yaw = np.deg2rad(yaw)
    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.083
	l07 = 0.083
	l08 = 0.082    
	l09 = 0.0535
	l10 = 0.059   # thickness of aluminum plate is around 0.006

	xg = xwg + 0.149
	yg = ywg - 0.149
	zg = zwg - 0.0193

	#print("****************\n****************\nGRIPPER LOCATION\n****************\n****************\n" + str((xg,yg,zg)) + "\n\n\n")

	xcen = xg - l09*np.cos(yaw)
	ycen = yg - l09*np.sin(yaw)
	zcen = zg

	# theta1
	#print("np.arcsin((l02-l04+l06) / np.sqrt(xcen*xcen + ycen*ycen))  : " + str(np.arcsin((l02-l04+l06) / np.sqrt(xcen*xcen + ycen*ycen))  ))
	#print("np.arctan2(ycen, xcen): " + str(np.arctan2(ycen, xcen)))
	
	thetas[0] = np.arctan2(ycen, xcen) - np.arcsin((l02-l04+l06) / np.sqrt(xcen*xcen + ycen*ycen))        # Default value Need to Change
	
	#print("Theta 1: " + str(thetas[0]))
	
	# theta6
	# thetas[5] = PI/2 + thetas[0] - yaw     # Default value Need to Change
 	
	thetas[5] = PI - yaw - (PI/2 - thetas[0])

 	#print("Theta 6: " + str(thetas[5]))
	
	x3end = (l06 + 0.027) * np.sin(thetas[0]) - l07 * np.cos(thetas[0]) + xcen
	y3end = -(l06 + 0.027) * np.cos(thetas[0]) - l07 * np.sin(thetas[0]) + ycen
	z3end = zcen + l08 + l10

	# some changes made, double check before demo
	c = np.sqrt((x3end**2 + y3end**2) + (z3end - l01)**2)
	alpha = np.arccos((l03**2 + c**2 - l05**2) / (2*l03*c))
	
	gamma = np.arcsin((z3end - l01) / c)

	beta = np.arcsin((np.sin(alpha)*c)/l05)

	thetas[1] = -(alpha + gamma) 				
	thetas[2] = PI - beta						
	thetas[3] = -(thetas[2] + thetas[1])        # need PI/2 for compensation
	thetas[4]= -PI/2							

	return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), 
					float(thetas[3]), float(thetas[4]), float(thetas[5]))

