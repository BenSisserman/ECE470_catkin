#!/usr/bin/env python

import sys
import copy
import time
import rospy

import numpy as np
from lab6_header import *
from lab6_func import *
from blob_search import *


# ============================= Student's code starts here ===================================

# Position for UR3 not blocking the camera 
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 130*PI/180.0]

# Store global coordinates of red & green blocks
xw_yw_R = []
xw_yw_G = []

red_block_target_location = [[0.14, -0.16, 0.10], [0.14, -0.10, 0.10]]
green_block_target_location = [[0.26, -0.10, 0.10], [0.26, -0.16, 0.10]]

green_pos_1 = np.radians([112.6, -52, 108.2, -148.6, -90, 44.6])
green_pos_2 = np.radians([116.4, -57.5, 116.5, -149, -90, 44.6])

pink_pos_2 = np.radians([130, -43.5, 87.3, -134.5, -90, 41])
pink_pos_1 = np.radians([125.1, -37.5, 73.8, -127.1, -90, 36.4])

green_tar_pos = [green_pos_1, green_pos_2]
pink_tar_pos = [pink_pos_1, pink_pos_2]


num_red_collected = 0
num_green_collected = 0

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = [120*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False
suction = True
"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

	global digital_in_0
	global analog_in_0
	digital_in_0 = msg.DIGIN
	digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
	analog_in_0 = msg.AIN0

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

"""
Function to control the suction cup on/off
"""
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

			#rospy.loginfo("Goal is reached!")
			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

"""
DONE: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg): 

	global suction

	# get data from msg

	suction = msg.DIGIN 
	suction = suction & 1



"""
Move robot arm from one position to another
"""
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

"""
Program run from here
"""
def main():

	global home

	# Initialize ROS node
	rospy.init_node('lab6node')

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)
	sub_gripper = rospy.Subscriber('ur3/gripper_input',gripper_input, gripper_callback)


	vel = 4.0
	accel = 4.0

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)




	'''CODE GOES HERE'''

	move_arm(pub_command, loop_rate, go_away, vel, accel)

	



if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass






################ Pre-defined parameters and functions no need to change below ################

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False


"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

	global digital_in_0
	digital_in_0 = msg.DIGIN
	digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0


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


"""
Function to control the suction cup on/off
"""
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

			#rospy.loginfo("Goal is reached!")
			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error


"""
Move robot arm from one position to another
"""
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

################ Pre-defined parameters and functions no need to change above ################


def move_block(pub_cmd, loop_rate, start_xw_yw_zw, end_pos, vel, accel):

	global suction

	error = 0	

	start_pos = lab_invk(start_xw_yw_zw[0], start_xw_yw_zw[1], start_xw_yw_zw[2], 0)
	#print(start_pos)
	
	if(isinstance(start_pos, int)):
		print("INVALID POSITION. IK CANNOT REACH CUBE AT POSITION " + str(start_xw_yw_zw) + ".\n")
		return -1
	
	#print(start_xw_yw_zw)
	#end_pos = #lab_invk(target_xw_yw_zw[0], target_xw_yw_zw[1], target_xw_yw_zw[2], 0)
	home = np.radians([165, -90, 90, -90, -90, 0])
	
	# start at home
	move_arm(pub_cmd, loop_rate, home, 5, 2)

	time.sleep(.1)
	
	# move arm to block location
	move_arm(pub_cmd, loop_rate, start_pos, vel, accel)
	time.sleep(0.1)

	#turn gripper on
	gripper(pub_cmd, loop_rate, 1)
	time.sleep(0.3)

	# suction cup feedback

	if(not suction):
		gripper(pub_cmd, loop_rate, 0)
		move_arm(pub_cmd, loop_rate, home, vel, accel)
		print("No block at src.")
		error = 1
		return error
	
	# move home to avoid knocking over blocks
	move_arm(pub_cmd, loop_rate, home, vel, accel)
	
	# move to dest
	move_arm(pub_cmd, loop_rate, end_pos, vel, accel)
	time.sleep(0.1)
	
	# turn gripper off
	gripper(pub_cmd, loop_rate, 0)


	# end at home
	move_arm(pub_cmd, loop_rate, home, vel, accel)
	
	return error


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.loop_rate = rospy.Rate(SPIN_RATE)

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
        	print("ROS is shutdown!")


    def image_callback(self, data):

        global xw_yw_R # store found red blocks in this list
        global xw_yw_G # store found green blocks in this list

        try:
		  # Convert ROS image to OpenCV image
            raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.flip(raw_image, -1)
        cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

		# You will need to call blob_search() function to find centers of red blocks
		# and green blocks, and store the centers in xw_yw_R & xw_yw_G respectively.

		# If no blocks are found for a particular color, you can return an empty list,
		# to xw_yw_R or xw_yw_G.

		# Remember, xw_yw_R & xw_yw_G are in global coordinates, which means you will 
		# do coordinate transformation in the blob_search() function, namely, from 
		# the image frame to the global world frame. 
		
        xw_yw_R = blob_search(cv_image, "red")
        xw_yw_G = blob_search(cv_image, "green")






"""
Program run from here
"""
def main():

	# global variable1
	# global variable2

	global go_away
	global xw_yw_R
	global xw_yw_G
	global red_block_target_location
	global green_block_target_location
	global num_green_collected
	global num_red_collected

	# Initialize ROS node
	rospy.init_node('lab6node')

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)

	vel = 4.0
	accel = 4.0
	move_arm(pub_command, loop_rate, go_away, vel, accel)

	ic = ImageConverter(SPIN_RATE)
	time.sleep(1)

	# ================================= Student's code starts here ===============================

	"""
	Hints: use the found xw_yw_R, xw_yw_G to move the blocks correspondingly. You will 
	need to call move_block(pub_command, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel)
	"""


	''' comment:
	print("\nRed:" + str(xw_yw_R))

	print("\nGreen:" + str(xw_yw_G))
	end comment '''


	green_locations = xw_yw_G
	red_locations = xw_yw_R


	# Green blocks
	while(len(green_locations) != 0):
		coords = green_locations.pop(0)
		coords.append(0.04)

		#print(coords)

		move_block(pub_command, loop_rate, coords, green_tar_pos[num_green_collected], vel, accel)
		
		num_green_collected += 1


	# Yellow blocks
	while(len(red_locations) != 0):
		coords = red_locations.pop(0)
		coords.append(0.04)

		move_block(pub_command, loop_rate, coords, pink_tar_pos[num_red_collected], vel, accel)
		
		num_red_collected += 1


	# ================================= Student's code ends here =================================

	move_arm(pub_command, loop_rate, go_away, vel, accel)
	rospy.loginfo("Task Completed!")
	print("Use Ctrl+C to exit program")
	rospy.spin()

if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass

