#!/usr/bin/env python

import cv2
import numpy as np 

"""
To init blob search params, will be init (called) in the ImageConverter class
"""
def blob_search_init():

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()

	################# Your Code Start Here #################

	# Filter by Color 
	params.filterByColor = True
	params.blobColor = 255
	

	# Filter by Area.
	params.filterByArea = True
	params.minArea = 100
	params.maxArea = 500


	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.15
	params.maxCircularity = 1

	# Filter by Inerita
	params.filterByInertia = True
	params.minInertiaRatio = 0.15

	
	# Filter by Convexity
	params.filterByConvexity = True
	params.minConvexity = 0.15
	
	


	################## Your Code End Here ##################

	# Create a detector with the parameters
	blob_detector = cv2.SimpleBlobDetector_create(params)

	return blob_detector


"""
To find blobs in an image, will be called in the callback function of image_sub subscriber
"""
def blob_search(image, detector):

	# Convert the color image into the HSV color space
	hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


	############################ Your Code Start Here ############################

	# Find lower & upper for orange
	"""
	lower =(110,50,50)      # blue lower
	upper = (130,255,255)   # blue upper
	
	
	H_max = 11
	H_avg = 8.94318181818
	H_min = 6
	S_max = 255
	S_avg = 253.90625
	S_min = 167
	V_max = 199
	V_avg = 188.426136364
	V_min = 119
	color_h = [  18.94318182  255.          255.        ]
	color_l = [   3.94318182  227.          119.        ]

	upper = (  20,  255, 255)
	lower = ( 3,  220, 110)


	"""

	upper = (30, 255,  255)
	lower = (10,60,0)

	############################# Your Code End Here #############################


	# Define a mask using the lower and upper bounds of the orange color 
	mask_image = cv2.inRange(hsv_image, lower, upper)

	crop_top_row = 100
	crop_bottom_row = 350
	crop_top_col = 150
	crop_bottom_col = 500

	crop_image = mask_image[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]

	blob_image_center = []

	############################ Your Code Start Here ############################

	# Call opencv simpleBlobDetector functions here to find centroid of all large enough blobs in 
	# crop_image. Make sure to add crop_top_row and crop_top_col to the centroid row and column found

	# Make sure this blob center is in the full image pixels not the cropped image pixels


	# Draw centers on each blob, append all the centers to blob_image_center as string in format "x y"

	keypoints = detector.detect(crop_image)
	kp_list = []

	for i in range(len(keypoints)):
		x = keypoints[i].pt[0] + crop_top_col
		y = keypoints[i].pt[1] + crop_top_row
		new_tuple = (x, y)
		print(new_tuple)
		keypoints[i].pt = new_tuple
		cv2.circle(image,(int(np.round(x)),int(np.round(y))), 1, (255, 255, 255), -1)
		#print((int(np.round(x)),int(np.round(y))))
		blob_image_center.append(str(int(np.round(x))) + " " + str(int(np.round(y))))
	
	im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	

	############################# Your Code End Here #############################

	# Draw small circle at pixel coordinate crop_top_col, crop_top_row so you can move a color
	# under that pixel location and see what the HSV values are for that color. 
	image = cv2.circle(image, (int(crop_top_col), int(crop_top_row)), 3, (0, 0, 255), -1)
	print('H,S,V at pixel ' + str(crop_top_row) + ' ' + str(crop_top_col) + ' ' + str(hsv_image[crop_top_row,crop_top_col]))	

	cv2.namedWindow("Maze Window")
	cv2.imshow("Maze Window", im_with_keypoints)

	cv2.namedWindow("MaskImage Window")
	cv2.imshow("MaskImage Window", mask_image)

	cv2.namedWindow("Crop Window")
	cv2.imshow("Crop Window", crop_image)

	cv2.waitKey(2)

	return blob_image_center
