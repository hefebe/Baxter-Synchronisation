#!/usr/bin/python
import numpy as np
import math
import os
import sys
import string
import time
import random
import tf
import struct

import cv;
import cv2;
from cv_bridge import CvBridge, CvBridgeError

import argparse

import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION

from sensor_msgs.msg import Image

from baxter_synchronisation.msg import Pos

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

#from baxter_pykdl import baxter_kinematics

S_Const = 100# lower limit of bandpass
I_Const = 5#upper limit of bandpass
Rate = 0.01


class slave:
	def __init__(self, start, speed, kay, scaling=0.05):# takes initial conditions and time constant
		
		self.state = start# state in x, y and z
		self.feedback = np.zeros(3)# feedback coming back from the plant
		self.power = 1# final model defaults to zero, although master feedback could force this
		
		self.state = np.array(start)# set the state
		
		self.rate = speed# set the rate

		self.DC = np.matrix(np.zeros(3))
		
		self.K = kay# this is the coupling constant

		self.scaling = scaling

		self.output = np.zeros(3)


	def update(self, mast):

		#self.state = np.array(self.state) +self.rate*(self.power-np.array(self.feedback))+ self.K*(self.scaling*np.array(mast) - self.output*self.scaling)

		#self.feedback = self.feedback + self.rate*(self.state)-self.K*(self.scaling*mast - self.output*self.scaling)# feedback term in the oscillator
		
		self.state = np.array(self.state) -self.rate*(np.array(self.state))+ self.K*(self.scaling*np.array(mast) - self.output*self.scaling)

		self.DC = self.DC + (self.rate/10)*(mast-self.output)
		self.output = self.state# + self.DC
		print self.output
		return self.output





class buff:# buffers values and performs noise filtering
	def __init__(self, lo, hi = 5):
		self.lo = lo
		self.hi = hi
		if lo > 0 and lo>=hi:
			self.wait = np.zeros((3,self.lo), dtype = np.float64)# maximum index is self.length-1
		else:
			self.wait = np.zeros((3,1), dtype = np.float64)
		self.counter = 1# counts number of iterations
		self.tell = np.zeros((3,1), dtype = np.float64)# vector of new coordinates

	def filterize(self, vPoint):
		self.tell = vPoint# pass new coordinates
		outer = np.zeros((3,1), dtype = np.float64)
		if self.lo ==0:# zero length indicates no filtering desired
			return np.array([[vPoint[0]], [vPoint[1]], [vPoint[2]]], dtype = np.float64)
		else:
			if(self.counter < self.lo):
				self.counter += 1# increment counter
			if not(np.isnan(self.tell).any()):# if the spherical transformation has not produced NaN (anything+NaN = NaN)
				self.wait[:, 1:self.lo] = self.wait[:, 0:self.lo-1]# shuffle up buffer - indices ARE CORRECT
				self.wait[:,0] = self.tell# add newest member #CHECK IF ALIGNED
		
		
				pus = np.sum(self.wait[:,0:self.lo], axis=1, dtype = np.float64)#self.pas = self.pas+np.reshape(self.tell,(3,1))# add to accumulator

				summed = np.sum(self.wait[:,0:self.hi], axis=1, dtype = np.float64)/self.hi
				outer = np.reshape(summed,(3,1))- np.reshape(pus,(3,1))/self.counter# in principle, rolling average should be updated after being used
			
			

			return outer# return high-pass filtered signal

class vMaster:
	def __init__(self, cropping = 0):# takes initial conditions and time constant

		self.crop = cropping

		self.left = [0, 0, 0]# left state in x, y and z
		self.L_left = [0, 0, 0]# last left state

		self.right = [0, 0, 0]# right state in x, y and z
		self.L_right = [0, 0, 0]# last right state

		self.feedback = np.zeros(3)# feedback coming back from the plant

		self.cam_calib    = 0.0025 
		
		self.width        = 320# Horizontal camera resolution
        	self.height       = 200# Vertical camera resolution
		
		self.xMax = 1.3# maximum arm span in metres
		self.yMax = 1# vertical arm reach in metres
		
		self.hough_accumulator = 30# smaller this is, the more false circles may be returned - circles with the largest values returned first
        	self.hough_min_radius  = 1# this will correspond to about golf ball radius, so change as appropriate
        	self.hough_max_radius  = 30

		self.cv_image = np.zeros((self.width, self.height, 3), np.uint8)# allocate a space for the camera image
		self.last_image = np.zeros((self.height, self.width, 3), np.uint8)
		self.L_image = np.zeros((self.width/2, self.height, 3), np.uint8)
		self.R_image = np.zeros((self.width/2, self.height, 3), np.uint8)
		
		self.dist = 1# standard distance
		self.rad = 40# radius of marker at D metres (pixels)

		self.bridge = CvBridge()# bridge object that allows me to convert image messages back and forth from CV

		self.headPub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

		self.begin = 0
		
		blobParams = cv2.SimpleBlobDetector_Params()# parameters
		blobParams.filterByInertia = 0# filter by squashiness/elongation
		blobParams.filterByConvexity = 0
		#blobParams.minInertiaRatio = 0.1# needs to be fairly blobby
		#blobParams.maxInertiaRatio = 1# basically a sphere
		blobParams.filterByArea = 1
		blobParams.minArea = 20
		blobParams.maxArea = 80


		
		self.detector = cv2.SimpleBlobDetector(blobParams)# set up detection
		

	def update(self):
		self.R_image = self.cv_image[0:self.height,0:self.width/2,:]# split image down the middle
		blb = self.hough_extractor("right")# extract best left circle
		if any(q != -1 for q in blb):# if it returns any blobs
			self.right[2] = self.ranger(blb[2])
			(self.right[0],self.right[1]) = self.converter(blb,self.right[2], 'right')

			self.right = blb
			self.L_right = self.right
			cv2.circle(self.R_image, (blb[0], blb[1]), blb[2], (0, 0, 255), 2)
		else:
			self.right = self.L_right# if there is no valid value, use the last one
		

		self.L_image = self.cv_image[0:self.height,self.width/2:self.width,:]# split image down the middle
		blb = self.hough_extractor("left")# extract best right circle
		if any(p != -1 for p in blb):
			self.left[2] = self.ranger(blb[2])
			(self.left[0],self.left[1]) = self.converter(blb,self.left[2], 'left')


			self.left = blb
			self.L_left = self.left
			cv2.circle(self.L_image, (blb[0], blb[1]), blb[2], (0, 0, 255), 2)
		else:
			self.left = self.L_left

		# zip left and right images back together
		self.cv_image[0:self.height,0:self.width/2,:] = self.R_image
		self.cv_image[0:self.height,self.width/2:self.width,:] = self.L_image

		
		#IM = self.bridge.cv2_to_imgmsg(self.cv_image, encoding="bgr8")# give image to head screen
		#self.headPub.publish(IM)# display it
		cv2.imshow('Head View',self.cv_image)
		cv2.waitKey(1)

	def camera_callback(self, data):
		# Convert image from a ROS image message to a CV image
		try:
			#self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			#self.cv_image = (cv2.blur(cv2.flip(np.asarray(self.bridge.imgmsg_to_cv2(data, "bgr8")),1),(5,5)))
			self.cv_image = cv2.flip(np.asarray(self.bridge.imgmsg_to_cv2(data, "bgr8")),1)
			self.cv_image[1:self.crop+1, 1:self.crop+1, :] = 0
			self.cv_image[self.height-1-self.crop:self.height-1, self.width-1-self.crop:self.width-1, :] = 0
			#######################BACKGROUND SUBTRACTION - LIGHT NOT GOOD ENOUGH#####################################
			#if self.begin == 0:
			#	self.cv_background = cv2.cvtColor(cv2.flip(np.asarray(self.bridge.imgmsg_to_cv2(data, "bgr8")),1), cv2.COLOR_BGR2GRAY)
			#	self.begin = 1
			#self.cv_image = cv2.flip(np.asarray(self.bridge.imgmsg_to_cv2(data, "bgr8")),1)
			#self.gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
			#diff = cv2.subtract(self.gray_image,self.cv_background)
			#ret,mask = cv2.threshold(diff,0,20,cv2.THRESH_BINARY_INV)
			#self.cv_image = cv2.bitwise_and(self.cv_image,self.cv_image,mask=mask)
			##########################################################################################################
			print('yes')
		except CvBridgeError, e:
			print e
		
		# 3ms wait
		#cv2.WaitKey(3)
		self.update()

	def converter(self, px, d, side):
		if side == 'right':
			x = (px[0])*(self.cam_calib * d * self.xMax / self.width)# get the horizontal coordinate (right counts from the middle, so it doesn't need to be centred)
		else:# I think normalisation is still on the same scale
			x = (px[0]-self.width/2)*(self.cam_calib * d * self.xMax / self.width)
		y = (px[1]-self.height/2)*(self.cam_calib * d * self.yMax / self.height)# get the vertical coordinate
        	return (x, y)# return tuple of x and y

	def ranger(self, radius):# find distance of the ball - this assumes the subject is approx. as tall as the robot
		distance = self.dist*radius/self.rad# multiply default distance by ratio of real vs. expected radius
		if distance > 0.5:# cap distance to within 'reach cuboid'
			distance = 0.5
		elif distance < 0:# prevent robot reacharound
			distance = 0
		
		distance = 1.5
		return distance

############################################ HOUGH EXTRACTOR ###############################################################
    	def hough_extractor(self, side):
		
		lower_blue = np.array([90,50,50])
    		upper_blue = np.array([150,255,255])

        	# create space for a single channel blue image (the robot is red, so I don't want to use that)
		if side == "left":# the image is divided down the middle, so only look at appropriate side

			hsv = cv2.cvtColor(self.L_image, cv2.COLOR_BGR2HSV)# DEPRECATED
			blue_image = cv2.inRange(hsv, lower_blue, upper_blue)
			self.L_image = cv2.bitwise_and(self.L_image,self.L_image, mask= blue_image)
			#blue_image = self.L_image[:,:,0]
			#blue_image = cv2.adaptiveThreshold(blue_image,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,-1)
			#self.L_image[:,:,0] = blue_image
			detect_image = self.L_image
		else:
			hsv = cv2.cvtColor(self.R_image, cv2.COLOR_BGR2HSV)
			blue_image = cv2.inRange(hsv, lower_blue, upper_blue)
			self.R_image = cv2.bitwise_and(self.R_image,self.R_image, mask= blue_image)
			#blue_image = self.R_image[:,:,0]
			#blue_image = cv2.adaptiveThreshold(blue_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,-1)
			#self.R_image[:,:,0] = blue_image
			detect_image = self.R_image
			
################################################################################################################
		
		#points = self.detector.detect(gray_image)# detect the points

		#if points == []:
			#blobbest = []
		#else:
			#blobbest = points[0]

			#if side == "left":# draw the best blob
				#self.L_image=cv2.drawKeypoints(self.L_image, points, np.array([]), (0,0,255), 	cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			
			#else:
				#self.R_image=cv2.drawKeypoints(self.R_image, points, np.array([]), (0,0,255), 	cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
################################################################################################################

		gray_image = cv2.cvtColor(detect_image, cv2.COLOR_BGR2GRAY)
		circles = cv2.HoughCircles(gray_image, cv.CV_HOUGH_GRADIENT, 4, 40, param1=100, param2=self.hough_accumulator, minRadius=self.hough_min_radius, maxRadius=self.hough_max_radius)
		
		if circles is None:# if there are no circles return error value
			c = np.asarray([-1, -1, -1])
		elif side == "left":# else return the best-fitting circle
			c= circles[0,0]
			cv2.circle(self.L_image, (c[0], c[1]), c[2], (0, 0, 255), 2)
		else:
			c= circles[0,0]
			cv2.circle(self.R_image, (c[0], c[1]), c[2], (0, 0, 255), 2)
		#c = np.asarray([-1, -1, -1])
		return c


		
############################################################################################################################


class Master:
	def __init__(self, start, speed):# takes initial conditions and time constant
		self.state = start# state in x, y and z
		self.mast = np.zeros(3)# state of the master
		self.feedback = np.zeros(3)# feedback coming back from the plant
		self.power = 1

		self.state = start# set the state
		self.rate = speed# set the rate

		self.metro = 0# "metronome"
		self.mFeed = 0

		#self.C = 0.1

		# self.a = 0.15# Rossler constants
		# self.b = 0.2
		# self.c = 10

	def update(self):
		self.mFeed = self.mFeed + self.rate*0.2*(self.metro)
		self.metro = self.metro + self.rate*0.2*(self.power-self.mFeed)
		
		self.feedback[0] = self.feedback[0] + self.rate*(self.state[0])
		self.feedback[1] = self.feedback[1] + self.rate*(self.state[1])
		self.feedback[2] = self.feedback[2] + self.rate*(self.state[2])

		self.state[0] = self.state[0] + self.rate*(1)*(self.power-self.feedback[0])#+np.random.random_sample()
		self.state[1] = self.state[1] + self.rate*(1)*(self.power-self.feedback[1])
		self.state[2] = self.state[2] + self.rate*(1)*(self.power-self.feedback[2])
		

	def polarize(self,point):
		radius = np.sqrt(np.power(point[0],2) + np.power(point[1],2) + np.power(point[2],2))# radial (extension) coordinate
		theta = np.arccos(point[2]/radius)# rotation around horizontal
		azimuth = np.arctan(point[1]/point[0])# rotation around vertical
		poles = np.array([radius, theta, azimuth])# put 'em together

		return poles

	def comZip(self, limb, osc, gain, vgain, side):# packages the states of the oscillators into a velocity command for the robot
		current = limb.joint_angles()# get the angles at each joint
		vels = limb.joint_velocities()# get the velocities

		diff = gain*np.array([osc[0], osc[2]-osc[1], 1*osc[0], osc[2]-osc[1], 0, 0, 0]) - vgain*np.array([vels[side+'_s0'], vels[side+'_s1'], vels[side+'_e0'], vels[side+'_e1'], 0, 0, 0])# this has been altered to take spherical error values directly, bypassing the dynamics

		labels = [side+'_s0', side+'_s1', side+'_e0', side+'_e1', side+'_w0', side+'_w1', side+'_w2']# labels for each joint
		comm = {labels[0]:diff[0], labels[1]:diff[1], labels[2]:diff[2], labels[3]:diff[3], labels[4]:diff[4], labels[5]:diff[5], labels[6]:diff[6]}

	#if any(t > 3 for t in diff):# going way too fast!! don't let the robot crash
		#print("TOO FAST")
		#sys.exit()# bug out
		return comm



def main():
	global S_Const
	global I_Const
	global rate

	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                  	description=main.__doc__)
	required = parser.add_argument_group('required arguments')# this section is adding the required arguments
	parser.add_argument(
    	"-C", "--cropping", type=float, default=0,# this one is a float, but the next piece of code checks its value is within bounds
    	help=("Window Cropping")
	)
	args = parser.parse_args(rospy.myargv()[1:])

	crop = args.cropping

####################### ENABLE FOR VISION ##########################################################################################
	#vmaster_targ = vMaster()

	#camera = baxter_interface.camera.CameraController("head_camera")# open an interface to the (now active) head cam
	#camera.open()
	#camera.resolution = (vmaster_targ.width, vmaster_targ.height)
	#camera.gain = 30

	
	#camera_sub = rospy.Subscriber("/cameras/head_camera/image", Image, vmaster_targ.camera_callback, queue_size = 1, buff_size = 12000)# subscribes to head image, but I need to make sure this isn't in synchronous mode
####################################################################################################################################
	
	#test_slave = slave(np.zeros(3), 0.05, 0.1, 1)# RE-ENABLE for vision!!!!

###################### DISABLE FOR VISION ##########################################################################################
	starter = [0, 0, 0]
	master_targ = Master(starter,Rate)
####################################################################################################################################

	rospy.init_node('master_unit')# init node

	Master_pub = rospy.Publisher('Master_state', Pos, queue_size=1)# I think I want to drop most missed messages, rather than let them pile up
	master_buff = buff(S_Const, I_Const)# buffer for the master value

	right_pub = rospy.Publisher('right_master', Pos, queue_size=1)# I think I want to drop most missed messages, rather than let them pile up
	right_buff = buff(S_Const)# buffer for the master value

	left_pub = rospy.Publisher('left_master', Pos, queue_size=1)# I think I want to drop most missed messages, rather than let them pile up
	left_buff = buff(10)# buffer for the master value

	Right = baxter_interface.Limb('right')# Right Arm!!
	Left = baxter_interface.Limb('left')# Left Arm!!

	#offset = [0.58, -0.3, 0.1]
	pose = Left.endpoint_pose()
	offset = np.array([pose['position'].x, pose['position'].y, pose['position'].z])

	scale = 0.05# scale for testing

	
	counter = 0
	r = rospy.Rate(100)
	while not rospy.is_shutdown():
		
		right_pose = Right.endpoint_pose()

		master_targ.update()
		
		smoothM = master_buff.filterize(master_targ.state)

		#smoothM = np.array([pose['position'].x, pose['position'].y, pose['position'].z])

########################################### RE-ENABLE for vision!!!! ####################################
		#smoothR = right_buff.filterize(vmaster_targ.right)
		#print smoothR
		#RM = test_slave.update(smoothR)
		#right_pub.publish(Pos(RM[0], RM[1], RM[2]))
		#left_pub.publish(Pos(smoothR[0], smoothR[1], smoothR[2]))
#########################################################################################################

		

		#smoothL = left_buff.filterize(vmaster_targ.left)


		poz = np.array([smoothM[0], smoothM[1], smoothM[2]])#-offset['position'].x
		

		posy = np.multiply(poz,[[scale*4], [scale*0.1], [scale*0.1]])#+master_targ.metro*scale#+offset
		print posy[0]
		
		if (counter > S_Const):
			Master_pub.publish(Pos(posy[0], posy[1], posy[2]))#CHECKCHECK
		counter +=1
		#Master_pub.publish(Pos(poz[0], -poz[1], poz[2]))# allows arm to arm synchronisation (not mirrored)

		#m_output = master_targ.polarize(offset+master_targ.state)# test output
		#C = master_targ.polarize(np.array([pose['position'].x, pose['position'].y, pose['position'].z]))

		#command = master_targ.comZip(Right, scale*m_output-C, 2, 1, "right")

		#Right.set_joint_velocities(command)# test movement to compare against slave
		r.sleep()
	#serv.close()
	return 0

if __name__ == '__main__':
    sys.exit(main())
