#!/usr/bin/env python

import sys
import cv2
import copy
import time
import numpy as np 

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lab3_func import blob_search_init, blob_search


######################## Replace values below in Part2 of Lab3 ########################

# Params for camera calibration
theta = 0 
beta = 0
tx = 0
ty = 0

#######################################################################################


class ImageConverter:

    def __init__(self, SPIN_RATE):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera_node/image_raw", Image, self.image_callback)
        self.coord_pub = rospy.Publisher("/coord_center", String, queue_size=10)
        self.loop_rate = rospy.Rate(SPIN_RATE)
        self.detector = blob_search_init()

        # Check if ROS is ready for operation
        while(rospy.is_shutdown()):
        	print("ROS is shutdown!")


    def image_callback(self, data):

		global theta
		global beta
		global tx
		global ty

		try:
		    # Convert ROS image to OpenCV image
		    raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
		    print(e)

		# Flip the image 180 degrees
		cv_image = cv2.flip(raw_image, -1)

		# Draw a black line on the image
		cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

		# cv_image is normal color image
		blob_image_center = blob_search(cv_image, self.detector)

		if(len(blob_image_center) == 0):
			print("No blob found!")
			self.coord_pub.publish("")
		else:
			x = int(blob_image_center[0].split()[0])
			y = int(blob_image_center[0].split()[1])
			# print("Blob found! ({0}, {1})".format(x, y))

			################################ Your Code Start Here ################################

			# Given theta, beta, tx, ty, calculate the world coordinate of x,y namely xw, yw  







			################################# Your Code End Here ################################# 

			xy_w = str(xw) + str(' ') + str(yw)
			# print(xy_w)
			self.coord_pub.publish(xy_w)


def main():

    SPIN_RATE = 20 # 20Hz

    rospy.init_node('lab3ImageNode', anonymous=True)

    ic = ImageConverter(SPIN_RATE)

    try:
    	rospy.spin()
    except KeyboardInterrupt:
    	print("Shutting down!")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
