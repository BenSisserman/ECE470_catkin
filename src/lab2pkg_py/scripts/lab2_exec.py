#!/usr/bin/env python

import copy
import time
import rospy
import numpy as np
from lab2_header import *

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)


############## Your Code Start Here ##############

# Hanoi tower location 1

Q11 = np.radians([120, -56, 124, -158, -90, 0])
Q12 = np.radians([120, -64, 123, -148, -90, 0])
Q13 = np.radians([120, -72, 120, -137, -90, 0])

Q = [ [Q11, Q12, Q13], \
      [Q11, Q12, Q13], \
      [Q11, Q12, Q13] ]

















############### Your Code End Here ###############


thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

suction = True


############## Your Code Start Here ##############

"""
DONE: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""


def gripper_callback(msg): 

	global suction

	# get data from msg

	suction = msg.DIGIN 
	suction = suction & 1


		
	
		




############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas
	global current_position
	global current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE
	global thetas
	global current_io_0
	global current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0   
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas
	global SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
	           end_loc, end_height):
	global Q
	error = 0

	return error



















############### Your Code End Here ###############


def main():

	global home
	global Q
	global SPIN_RATE
	global suction

	tower1 = [ 
		# pose lowest cube in tower 1
		[2.7045328617095947, -0.6535270849810999, 1.3716363906860352, -2.309315029774801, -1.578137222920553, -0.0003584066974084976],
		# middle cube
		[2.702254056930542, -0.7386120001422327, 1.3725605010986328, -2.2435177008258265, -1.5506895224200647, -0.00032264391054326325],
   		# top cube
   		[2.7024340629577637, -0.8031662146197718, 1.3303685188293457, -2.1214025656329554, -1.5648816267596644, -0.0029051939593713882]
	]

	tower2 = [
		[2.949126720428467, -0.6304333845721644, 1.3306326866149902, -2.285335365925924, -1.539199177418844, -0.0029051939593713882],
		[2.9519577026367188, -0.7144530455218714, 1.3311004638671875, -2.222482983266012, -1.5490086714373987, -0.0029051939593713882],
		[2.951249837875366, -0.7914946714984339, 1.3036761283874512, -2.1237853209124964, -1.550089184437887, -0.0028336683856409195]
	]

	tower3 = [
		[3.201143503189087, -0.5264185110675257, 1.122664451599121, -2.248761002217428, -1.5501254240619105, -0.003144566212789357],
		[3.212289571762085, -0.608405892048971, 1.1217288970947266, -2.15994102159609, -1.5500171820269983, -0.0032160917865198257],
		[3.201491355895996, -0.6468876043902796, 1.0041594505310059, -1.9408190886126917, -1.5477479139911097, -0.003156487141744435]
	]

	# Initialize ROS node
	rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)

	############## Your Code Start Here ##############
	# DONE: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

	sub_gripper = rospy.Subscriber('ur3/gripper_input',gripper_input, gripper_callback)






	############### Your Code End Here ###############


	############## Your Code Start Here ##############
	# DONE: modify the code below so that program can get user input

	start_input_done = 0
	end_input_done = 0
	loop_count = 0

	while(not start_input_done):
		start_start_input_string = raw_input("Enter starting position of tower: <Either 1 2 or 3> ")
		print("You entered " + start_start_input_string + "\n")

		if(int(start_input_string) == 1):
			input_done = 1
			loop_count = 1
		elif (int(start_input_string) == 2):
			input_done = 1
			loop_count = 2
		elif (int(start_input_string) == 3):
			input_done = 1
			loop_count = 3
		else:
			print("Please just enter the character 1 2 or 3 \n\n")
			print("You entered " + start_input_string + "\n")

		if(int(start_input_string) == 1):
			input_done = 1
			loop_count = 1
		elif (int(start_input_string) == 2):
			input_done = 1
			loop_count = 2
		elif (int(start_input_string) == 3):
			input_done = 1
			loop_count = 3
		else:
			print("Please just enter the character 1 2 or 3 \n\n")

while(not end_input_done):
		start_end_input_string = raw_input("Enter starting position of tower: <Either 1 2 or 3> ")
		print("You entered " + start_end_input_string + "\n")

		if(int(end_input_string) == 1):
			input_done = 1
			loop_count = 1
		elif (int(end_input_string) == 2):
			input_done = 1
			loop_count = 2
		elif (int(end_input_string) == 3):
			input_done = 1
			loop_count = 3
		else:
			print("Please just enter the character 1 2 or 3 \n\n")
			print("You entered " + end_input_string + "\n")

		if(int(end_input_string) == 1):
			input_done = 1
			loop_count = 1
		elif (int(end_input_string) == 2):
			input_done = 1
			loop_count = 2
		elif (int(end_input_string) == 3):
			input_done = 1
			loop_count = 3
		else:
			print("Please just enter the character 1 2 or 3 \n\n")




	############### Your Code End Here ###############

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	rospy.loginfo("Sending Goals ...")

	loop_rate = rospy.Rate(SPIN_RATE)

	############## Your Code Start Here ##############
	# TODO: modify the code so that UR3 can move tower accordingly from user input

	while(loop_count > 0):

		move_arm(pub_command, loop_rate, home, 4.0, 4.0)

		rospy.loginfo("Sending goal 1 ...")
		move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

		gripper(pub_command, loop_rate, suction_on)
		# Delay to make sure suction cup has grasped the block
		time.sleep(1.0)
		if(suction == 0):
			print("Nothing gripped\n Shutting down and goind home.")
			break


		rospy.loginfo("Sending goal 2 ...")
		move_arm(pub_command, loop_rate, Q[0][1], 4.0, 4.0)

		rospy.loginfo("Sending goal 3 ...")
		move_arm(pub_command, loop_rate, Q[0][2], 4.0, 4.0)

		loop_count = loop_count - 1



	gripper(pub_command, loop_rate, suction_off)

	move_arm(pub_command, loop_rate, home, 4.0, 4.0)








	############### Your Code End Here ###############



if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass


	






