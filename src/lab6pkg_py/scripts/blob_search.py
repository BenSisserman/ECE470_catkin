#!/usr/bin/env python

import cv2
import numpy as np

# ============================= Student's code starts here ===================================

# Params for camera calibration
theta = 0 
beta = 740.0
tx = -0.282094594595
ty = -0.143581081081


# Function that converts image coord to world coord
def IMG2W(x,y):
	pass

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
	params.minArea = 200 #was 100
	params.maxArea = 500


	# Filter by Circularity
	params.filterByCircularity = True # was true
	params.minCircularity = 0 # 0.15
	params.maxCircularity = .77 # 1

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
def blob_search(image, color):

	# get detector
	detector = blob_search_init()

	# Convert the color image into the HSV color space
	hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


	############################ Your Code Start Here ############################



	# GREEM
	upper_green = (70, 225, 225)
	lower_green = (50, 150, 125)

	#PIBK
	upper_pink =  (15, 255, 255)
	lower_pink =  ( 0, 200, 220)


	# Orange HSV
	upper_orange = (30, 255,  255)
	lower_orange = (10,60,0)

	############################# Your Code End Here #############################


	# Define a mask using the lower and upper bounds of the orange color 
	mask_image = cv2.inRange(hsv_image, lower_orange, upper_orange)

	green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

	pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)


	crop_top_row = 100
	crop_bottom_row = 350
	crop_top_col = 150
	crop_bottom_col = 500

	crop_image = mask_image[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]
	green_crop_image = green_mask[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]
	pink_crop_image = pink_mask[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]

	blob_image_center = []
	green_center = []
	pink_center = []
	############################ Your Code Start Here ############################

	# Call opencv simpleBlobDetector functions here to find centroid of all large enough blobs in 
	# crop_image. Make sure to add crop_top_row and crop_top_col to the centroid row and column found

	# Make sure this blob center is in the full image pixels not the cropped image pixels


	# Draw centers on each blob, append all the centers to blob_image_center as string in format "x y"


	# ORANGE
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


	# GREEN
	keypoints = detector.detect(green_crop_image)
	kp_list = []

	for i in range(len(keypoints)):
		x = keypoints[i].pt[0] + crop_top_col
		y = keypoints[i].pt[1] + crop_top_row
		new_tuple = (x, y)
		print(new_tuple)
		keypoints[i].pt = new_tuple
		cv2.circle(im_with_keypoints,(int(np.round(x)),int(np.round(y))), 1, (255, 255, 255), -1)
		#print((int(np.round(x)),int(np.round(y))))
		green_center.append(str(int(np.round(x))) + " " + str(int(np.round(y))))
	
	im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


	# PINK
	keypoints = detector.detect(pink_crop_image)
	kp_list = []

	for i in range(len(keypoints)):
		x = keypoints[i].pt[0] + crop_top_col
		y = keypoints[i].pt[1] + crop_top_row
		new_tuple = (x, y)
		print(new_tuple)
		keypoints[i].pt = new_tuple
		cv2.circle(im_with_keypoints,(int(np.round(x)),int(np.round(y))), 1, (255, 255, 255), -1)
		#print((int(np.round(x)),int(np.round(y))))
		pink_center.append(str(int(np.round(x))) + " " + str(int(np.round(y))))
	
	im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	

	############################# Your Code End Here #############################

	# Draw small circle at pixel coordinate crop_top_col, crop_top_row so you can move a color
	# under that pixel location and see what the HSV values are for that color. 
	image = cv2.circle(image, (int(crop_top_col), int(crop_top_row)), 3, (0, 0, 255), -1)
	print('H,S,V at pixel ' + str(crop_top_row) + ' ' + str(crop_top_col) + ' ' + str(hsv_image[crop_top_row,crop_top_col]))	
	
	cv2.namedWindow("Maze Window")
	cv2.imshow("Maze Window", im_with_keypoints)
	'''
	cv2.namedWindow("MaskImage Window")
	cv2.imshow("MaskImage Window", mask_image)
	
	cv2.namedWindow("GREEN")
	cv2.imshow("GREEN", green_mask)
	
	cv2.namedWindow("PINK")
	cv2.imshow("PINK", pink_mask)
	

	cv2.namedWindow("Crop Window")
	cv2.imshow("Crop Window", crop_image)
	'''

	cv2.waitKey(2)

	if(color == "red"):
		return pink_center
	elif(color == "green"):
		return green_center
	return blob_image_center
	