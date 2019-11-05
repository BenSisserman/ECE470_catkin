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

	M = np.array([[0,-1,0,0.54], [0,0,-1,0.242], [1,0,0,0.212], [0,0,0,1]])
	S = np.zeros((6,6))

	w1 = np.array([0,0,1])
	w2 = np.array([0,1,0])
	w3 = np.array([0,1,0])
	w4 = np.array([0,1,0])
	w5 = np.array([1,0,0])
	w6 = np.array([0,1,0])

	q1 = np.array([0,0,0])
	q2 = np.array([0,0,0.152])
	q3 = np.array([0.244,0,0.152])
	q4 = np.array([0.457,0,0.152])
	q5 = np.array([0,0.11,0.152])
	q6 = np.array([0.54,0,0.152])

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
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	exp = []
	for i in range(6):
		print(i)
		print(S[i])
		print(theta[i])
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

	xg = xwg + 0.1485
	yg = ywg - 0.1485
	zg = zwg - 0.013

	

	xcen = xg - l09*np.cos(yaw)
	ycen = yg - l09*np.sin(yaw)
	zcen = zg

	# theta1
	thetas[0] = np.arctan2(ycen, xcen) - np.arcsin((l02-l04+l06) / np.sqrt(xcen*xcen + ycen*ycen))        # Default value Need to Change

	# theta6
	thetas[5] = PI/2 + thetas[0] - yaw     # Default value Need to Change
 	
	
	x3end = (l06 + 0.027) * np.sin(thetas[0]) - l07 * np.cos(thetas[0]) + xcen
	y3end = -(l06 + 0.027) * np.cos(thetas[0]) - l07 * np.sin(thetas[0]) + ycen
	z3end = zcen + l08

	c = np.sqrt(x3end*x3end + y3end*y3end + (z3end - l01)*(z3end - l01))
	# helper variables
	print(x3end)
	print(y3end)
	print(z3end)
	print(xcen)
	print(ycen)
	print(zcen)
	alpha = np.arccos((-c*c + l03*l03 + l05*l05)/(2*l03*l05))
	beta =  np.arcsin(l05 / c * np.sin(alpha))
	gamma = np.arcsin((z3end - l01) / c )
	etta = PI - alpha - beta
	delta = PI/2 - alpha 

	thetas[1]= -(beta + gamma)     				# Default value Need to Change
	thetas[2]= PI - alpha      					# Default value Need to Change
	thetas[3]= -(etta + delta) + (0.5*PI) 		# Default value Need to Change, need + (0.5*PI) for compensation
	thetas[4]= -PI/2      						# Default value Need to Change


	print(thetas)
	print("theta1 to theta6: " + str(thetas) + "\n")

	return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
		          float(thetas[3]), float(thetas[4]), float(thetas[5]) )
