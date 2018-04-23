#!/usr/bin/python
# alright, this one implements the slave target dynamics and interfaces
# with the robot simulation to produce the anticipated error
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
import cv_bridge

import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION

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

	
class Master:
	def __init__(self):# takes initial conditions and time constant
		self.left = [0, 0, 0]# left state in x, y and z
		self.right = [0, 0, 0]# right state in x, y and z
		self.feedback = np.zeros(3)# feedback coming back from the plant
		
		self.width        = 320# Horizontal camera resolution
        	self.height       = 200# Vertical camera resolution
		
		self.xMax = 1.6# maximum arm span in metres
		self.yMax = 1# vertical arm reach in metres
		
		self.hough_accumulator = 35# smaller this is, the more false circles may be returned - circles with the largest values returned first
        	self.hough_min_radius  = 7# this will correspond to about golf ball radius, so change as appropriate
        	self.hough_max_radius  = 20
		
		self.cv_image = cv.CreateImage((self.width, self.height), 8, 3)# allocate a space for the camera image
		
		self.dist = 1# standard distance
		self.scaling = 0.0025# constant I'll use to calibrate distance
		self.rad = 0.2# radius at distance D
		
	
	def update(self):
		left_hand = hough_extractor("left")# extract best left circle
		right = hough_extractor("right")# extract best right circle
		
		# x y and z for left ball/hand
		self.left[0] = left_hand[0]
		self.left[1] = left_hand[1]
		self.left[2] = left_hand[2]
		
		# x y and z for right ball/hand
		self.right[0] = right[0]
		self.right[1] = right[1]
		self.right[2] = right[2]
		
    	def cv2array(self, im):# this turns the cvimage im into a numpy array, via string - not sure if necessary
        	depth2dtype = {cv.IPL_DEPTH_8U: 'uint8',
                       cv.IPL_DEPTH_8S: 'int8',
                       cv.IPL_DEPTH_16U: 'uint16',
                       cv.IPL_DEPTH_16S: 'int16',
                       cv.IPL_DEPTH_32S: 'int32',
                       cv.IPL_DEPTH_32F: 'float32',
                       cv.IPL_DEPTH_64F: 'float64'}
  
        	arrdtype=im.depth
        	a = numpy.fromstring(im.tostring(),
                             dtype = depth2dtype[im.depth],
                             count = im.width * im.height * im.nChannels)
        	a.shape = (im.height, im.width, im.nChannels)

        	return a

    	def hough_extractor(self, side):
		
        	# create space for a single channel blue image (the robot is red, so I don't want to use that)
        	blue_image = cv.CreateImage((self.width/2, self.height), 8, 1)# half-width, for splitting down the middle
        	#cv.CvtColor(self.cv_image, blue_image, cv.CV_BGR2GRAY)
		
		if side = "left":# the image is divided down the middle, so only look at appropriate side
			cv.Split(self.left_image, blue_image)
		else:
			cv.Split(self.right_image, blue_image)

        	# create gray scale array of balls
        	blue_array = cv2array(blue_image)

        	# find Hough circles
        	circles = cv2.HoughCircles(blue_array, cv.CV_HOUGH_GRADIENT, 1, 40, param1=50,  \# param1 is the higher threshold for hough_gradient (lower is half that)
                  	param2=self.hough_accumulator, minRadius=self.hough_min_radius,       \
                  	maxRadius=self.hough_max_radius)

        	# Check for at least one ball found
        	if circles is None:
            	sys.exit("Failure to isolate hand positions")

        	circles = numpy.uint16(numpy.around(circles))# rounds into uint16 for nice image-ness

        	ball_data = {}
        	n_balls   = 0

        	circle_array = numpy.asarray(self.cv_image)# turns something that is like an arry, into an array

        	# check if golf ball is in ball tray
        	i = circles[0,0]:
        	# convert to baxter coordinates
        	ball = self.pixel_to_baxter((i[0], i[1]), self.tray_distance)

        	if i[1] < 200:# perform range check to check if too distant
            	# draw the outer circle in red
            	cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
        	elif i[1] > 800:# check if too close
            	# draw the outer circle in red
            	cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 0, 255), 2)
        	else:# else if good
            	# draw the outer circle in green
            	cv2.circle(circle_array, (i[0], i[1]), i[2], (0, 255, 0), 2)

            	ball_data[n_balls]  = (i[0], i[1], i[2])
            	n_balls            += 1

        	circle_image = cv.fromarray(circle_array)

        	cv.ShowImage("Hough Circle", circle_image)

        	# 3ms wait
        	cv.WaitKey(3)

        	# display image on head monitor
        	font     = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 1)
        	position = (30, 60)
        	s = "Searching for golf balls"
        	cv.PutText(circle_image, s, position, font, self.white)
        	msg = cv_bridge.CvBridge().cv_to_imgmsg(circle_image, encoding="bgr8")
        	self.pub.publish(msg)


        	# Check for at least one ball found
        	if n_balls == 0:                    # no balls found
            	# display no balls found message on head display
            	self.splash_screen("no balls", "found")
            	# less than 12 balls found, no point in continuing, exit with error message
            	sys.exit("ERROR - hough_it - No golf balls found")

        	# find best ball on right, then iterate to find best on left(?)
        	hands = self.find_hands(ball_data, iteration)
		
        	# return next golf ball position and pickup angle
        	return next_ball
		    
			
			
    	def find_hands(self, ball_data, Z):# simply want to extract
		Z = self.ranger(ball_data[0][0])# get distance of ball
		xy = self.pixel_to_baxter(ball_data[0][0], Z)
		return ball_data[0]
		
			
	def camera_callback(self, data):
		# Convert image from a ROS image message to a CV image
		try:
			self.cv_image = cv_bridge.CvBridge().imgmsg_to_cv(data, "bgr8")
		except cv_bridge.CvBridgeError, e:
			print e
		
		# 3ms wait
		cv.WaitKey(3)
		self.update()
			
	def ranger(self, radius):# find distance of the ball - this assumes the subject is approx. as tall as the robot
		distance = self.dist*radius/self.rad# multiply default distance by ratio of real vs. expected radius
		return distance
		
    	def pixel_to_baxter(self, px, d):# transform pixel coordinates from the head cam into Baxter coordinates for the arms
		# x = ((px[1] - (self.height / 2)) * self.cam_calib * d * self.max_reach[0])/self.height
		# y = ((px[2] - (self.width / 2)) * self.cam_calib * d * self.max_reach[1])/self.width
		x = (px[0]-self.width/2)*(self.cam_calib * d * self.xMax / self.width)# get the horizontal coordinate
		y = (px[1]-self.height/2)*(self.cam_calib * d * self.yMax / self.height)# get the vertical coordinate
        	return (x, y)# return tuple of x and y
		
def main():
	master_targ = Master(np.array()# different initial condition
	
	rospy.init_node('master_unit')# init node
	Master_pub = rospy.Publisher('Master_state', Pos, queue_size=10)

	Right = baxter_interface.Limb('right')
	camera = baxter_interface.camera.CameraController("right_camera")
	camera.close()# this closes and powers down the camera on the right, which opens and powers up the other two
	camera = baxter_interface.camera.CameraController("head_camera")# open an interface to the (now active) head cam
	
	camera_sub = rospy.Subscriber("/cameras/head_camera/image", Image, Master.callback)# subscribes to head image, but I need to make sure this isn't in synchronous mode

	while not rospy.is_shutdown():
		master_targ.update()
		Master_pub.publish(Pos(master_targ.state[0], master_targ.state[1], master_targ.state[2]))
		
		#poser = ik_test(Right, slave_targ.state, 2, serv, NS)
		#Right.set_joint_positions(poser)
	return 0

if __name__ == '__main__':
    sys.exit(main())
