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

import argparse

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
import std_msgs

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

CARTESIAN = 1# if using cartesian control

depth = 4# number of joints recruited for movements
spread = 1# number of individual frequencies
downSample = 0# downsampling rate (2 = move every other loop)

S_Const = 10
I_Const = 1#upper limit of bandpass
	
class buff:# buffers values and performs noise filtering
	def __init__(self, length):# length is the length of the buffer, pas is upper limit of passband
		self.length = length
		if length:
			self.wait = np.zeros((3,self.length))
		else:
			self.wait = np.zeros((3,1))
		self.pas = np.zeros((3,1))
		self.counter = 0
		self.tell = np.zeros((3,1))
	def filterise(self, ePoint):
		self.counter += 1
		if self.length ==1:
			return np.array([ePoint['position'].x, ePoint['position'].y, ePoint['position'].z])
		self.wait[:, 1:self.length-1] = self.wait[:, 0:self.length-2]# shuffle up
		self.wait[:,0] = np.array([ePoint['position'].x, ePoint['position'].y, ePoint['position'].z])# add newest member
		
		out = np.sum(self.wait[:,0:self.length],1)/self.length - self.pas/self.counter
		self.pas += self.wait[:,0]# mean value
		return out# return high-pass filtered signal
	def filterize(self, vPoint):
		self.tell = vPoint
		
		if self.length ==0:
			return np.array([vPoint[0], vPoint[1], vPoint[2]])
		self.wait[:, 1:self.length] = self.wait[:, 0:self.length-1]# shuffle up
		self.wait[:,0] = self.tell# add newest member
		
		if (not(np.isnan(self.wait[:,0]).any())):
			self.pas = self.pas+np.reshape(self.wait[:,0],(3,1))# mean value
			self.counter += 1
		return np.reshape(np.sum(self.wait[:,0:self.length],1)/self.length,(3,1)) - self.pas/self.counter# return high-pass filtered signal


class armslave:
	def __init__(self, start, speed, kay, begin, side, scaling=0.05, length = 1, pas = 1, thresh = 0):# takes initial conditions and time constant

		global depth
		
		self.state = start# initial state in x, y and z
		self.mast = np.zeros(3)# state of the master (position in x, y and z)
		self.feedback = np.zeros(3)# feedback coming back from the plant
		self.power = 0# intrinsic driving term

		self.Cstate = np.zeros((3,spread))# TEST Cartesian internal model
		self.Cfeed = np.zeros((3,spread))
		
		self.state = np.array(start)# set the state

		self.multiState = np.zeros((depth,spread))# as many states as joints# becomes np.zeros((depth,spread)) (I think)
		self.multiFeed = np.zeros((depth,spread))# matching number of feedbacks# becomes np.zeros((depth,spread)) (I think)

		self.Cbuff = buff(length)# high pass filter for the coupling term to remove trends in the sensor noise (this is applied here because the noise sources in master and slave are independent)
		self.Dbuff = buff(5)# for DC
		self.Master_Mask = np.zeros((3,1))# boolean mask to determine if coupling can be ignored
		self.Master_Thresh = thresh# the value the master avg must be exceeding for coupling to 'kick in' - prevents system crosstalk

		self.DC = np.zeros((depth,spread))# DC components for each dimension
		self.connect = np.ones((spread,1))# adds up DC components
		
		#self.patch = np.zeros(3)# this is a term to correct visual DC

		self.mrate = np.linspace(speed, speed*spread, spread)# oscillators map directly to joints, so differing frequencies not necessary# this becomes a linspace of spread length

		self.ratio = 0# ratio between oscillation and 'DC component'/relaxation

		self.mK = np.linspace(kay, kay/spread, spread)# vector of coupling constant, currently all the same# this becomes a linspace of spread length#
		self.maxC = 0.1# the maximum value of the coupling term, designed to engender stability# DEPRECATE??

		self.mix = np.array([[0,0,1],[-1,1,0],[0,0,1],[-1,1,0]])#maps from dimensions to joints (JRM) - remember, keep coupling constant at equal power for each slave, including summed across sources

		self.interface = begin# arm interface allows for safe stopping

		self.initial = begin.endpoint_pose()# holds the initial pose of the robot
		
		self.K = kay# this is the coupling constant

		self.off = np.zeros(3)# offset that makes up for non-stationary noise# DEPRECATE??

		self.scaling = scaling# scale between the oscillator and real metre measurements

		self.Aoffset = np.array([-0.5, -1.2, 0.3, 0])# hardcoded offset to shift zero-centred oscillators onto true joint midpoints
		self.Arange = [[3], [2.4], [3.4], [3.2]]# scaling factor for joints (conversion to radians)

		self.side = side# the side in question

		#self.stacker = np.tril(np.ones(spread,spread))# represents progressive subtraction of slaves
		
	def add_master(self, data):# grab the state of the master
		#if data.x:# this just prevents an edge case where the coupling is too large
		self.mast[0] = data.x
		
		

		self.mast[1] = data.y

		
		
		self.mast[2] = data.z
		

		print("master")

	
	def update(self, poser, angles, side, init, current, setup, smooth = np.array([0,0,0])):# slave pose, master angles, side of master, initial angles of slave, current angles of slave, if in setup mode or not
		global depth
		global CARTESIAN
		
		otherSide = {'right':'left', 'left':'right'}# dictionary that reverses side term

		mAngles = np.array([[-angles[side+'_s0']], [angles[side+'_s1']], [-angles[side+'_e0']], [angles[side+'_e1']]])# master angles
		iAngles = np.array([[init[otherSide[side]+'_s0']], [init[otherSide[side]+'_s1']], [init[otherSide[side]+'_e0']], [init[otherSide[side]+'_e1']]])# slave initial angles

		if (self.mast.any() !=0):
			if (CARTESIAN == 1):
				smoothed = self.Cbuff.filterize(np.array([self.mast[0],self.mast[1],self.mast[2]])-[smooth[0],smooth[1],smooth[2]])# not sphericalized
			else:
				smoothed = self.Cbuff.filterize(self.polarize(np.array([self.mast[0],self.mast[1],self.mast[2]]))-self.polarize([smooth[0],smooth[1],smooth[2]]))# sphericated
		else:
			smoothed = np.zeros(3)
		smoothedD = self.Dbuff.filterize(self.polarize(np.array([self.mast[0],self.mast[1],self.mast[2]])))
		#smoothed = smoothedD-self.Cbuff.filterize(self.polarize([smooth[0],smooth[1],smooth[2]]-self.off))
		print smoothed
		self.Master_Mask = np.absolute(smoothedD) <= self.Master_Thresh# check if the running average of the master exceeds some magnitude
		if (np.isnan(smoothed).any() or np.isnan(smoothedD).any()):# make sure coupling exists
			inter = np.zeros([3,spread])
			interR = np.zeros([3,spread])

			couple = np.zeros([depth,spread])
			coupleR = np.zeros([depth,spread])
		else:
			interR = np.dot(np.reshape(smoothed,(3,1)), np.reshape(self.mK,(1,spread)))# get unaltered coupling
			#smoothed[self.Master_Mask]=0
			

			inter = np.dot(np.reshape(smoothed,(3,1)), np.reshape(self.mK,(1,spread)))# coupling multiplied by K vector
			

			couple = np.dot(self.mix, inter)# 3-by-4 mixing matrix multiplied by 4-by-1 vector to produce a 4-by-1 coupling vector
			
			coupleR = np.dot(self.mix, interR)
		if setup != 0:# if setup is over
			delta = np.multiply(self.power-self.multiFeed, self.mrate) + couple
			self.multiState += delta # update state dynamics, using individual time constants and coupling terms

			felta = np.multiply(self.multiState, self.mrate) - couple
			self.multiFeed += felta# feedback term for the oscillator

			#self.multiState += np.multiply(-self.multiState, self.mrate) # relaxation dynamics
			#self.multiState += couple

			dDelta = np.multiply(coupleR-self.DC, self.mrate[0]) + couple
			self.DC += dDelta#DC term represents changes in the master's average position# (mAngles-iAngles)
		#else:# if still in setup time
			#if all (a !=0 for a in self.mast):# provided the master exists...
				#self.patch = self.patch + np.multiply(0,np.array([self.mast[0],self.mast[1],self.mast[2]])-np.array([poser['position'].x, poser['position'].y, poser['position'].z])-self.patch)# 'patch' term converges on the initial difference between master and slave

			
			CDelta = np.multiply(-self.Cstate, self.mrate[0]) + [[inter[0]], [0], [0]]
			self.Cstate += CDelta#DC term represents changes in the master's average position# (mAngles-iAngles)

			#Cfelta = np.multiply(self.Cstate, self.mrate[0]) - [[inter[0]], [0], [0]]# attempt at oscillator
			#self.Cfeed += Cfelta


		output = np.multiply(self.Arange, np.array(self.scaling*np.dot(delta,self.connect)))*0 +np.dot(self.mix,CDelta)*(1-self.ratio) +np.dot(dDelta, self.connect)*self.ratio# scaling and offset#+iAngles# +np.dot(self.mix,CDelta)*0
		#uncomment to output conventional error term as well
		conventional = np.zeros(depth)
		if (not(np.isnan(inter).any())):# LAST CONFIGURATION TESTED -CHECK IT
			conventional = np.dot(couple,self.connect)

		#wee = np.transpose(couple)
		return output, smoothed, smooth, conventional, inter


	def polarize(self,point):
		radius = np.sqrt(np.power(point[0],2) + np.power(point[1],2) + np.power(point[2],2))# radial (extension) coordinate
		theta = np.arccos(point[2]/radius)# rotation around horizontal
		azimuth = np.arctan(point[1]/point[0])# rotation around vertical
		poles = np.array([radius, theta, azimuth])# put 'em together

		return poles

	def shut(self):# resets velocity and hopefully stops the robot bouncing around
		labels = [self.side+'_s0', self.side+'_s1', self.side+'_e0', self.side+'_e1', self.side+'_w0', self.side+'_w1', self.side+'_w2']# labels for each joint
		stopper = {labels[0]:0, labels[1]:0, labels[2]:0, labels[3]:0, labels[4]:0, labels[5]:0, labels[6]:0}# zero velocity command
		self.interface.set_joint_velocities(stopper)# stop

	

def comZip(limb, osc, gain, vgain, side, init, conv = np.zeros(depth), ratio = 1):# packages the states of the oscillators into a velocity command for the robot
	current = limb.joint_angles()# get the angles at each joint
	vels = limb.joint_velocities()# get the velocities

	pose = limb.endpoint_pose()

	#diff = gain*ratio*np.array([(osc[0]-(current[side+'_s0'])), (osc[1]-(current[side+'_s1'])), (osc[2]-(current[side+'_e0'])), (osc[3]-(current[side+'_e1'])),	0, 0, 0])

	diff = gain*ratio*np.array([(osc[0]), (osc[1]), (osc[2]), (osc[3]), 0, 0, 0])
	diffC = (1-ratio)*(gain*np.array([conv[0], conv[1], conv[2], conv[3], -0*(init['orientation'].y-pose['orientation'].y), 0*(init['orientation'].x-pose['orientation'].x), 0])-vgain*np.array([vels[side+'_s0'], vels[side+'_s1'], vels[side+'_e0'], vels[side+'_e1'], 0*vels[side+'_w0'], 0*vels[side+'_w1'], 0]))# viscosity # follow oscillator by calculating the difference with the joints current positions and transforming that into a velocity command## this has been altered to take spherical error values directly, bypassing the dynamics
	
	
	labels = [side+'_s0', side+'_s1', side+'_e0', side+'_e1', side+'_w0', side+'_w1', side+'_w2']# labels for each joint
	comm = {labels[0]:diff[0]+diffC[0], labels[1]:diff[1]+diffC[1], labels[2]:diff[2]+diffC[2], labels[3]:diff[3]+diffC[3], labels[4]:diff[4]+diffC[4], labels[5]:diff[5]+diffC[5], labels[6]:diff[6]+diffC[6]}

	if any(t > 1.5 for t in diff+diffC):# going way too fast!! don't let the robot crash
		print diff+diffC
		print("TOO FAST")
		sys.exit()# bug out
	return comm

def comZipOld(limb, osc, gain, vgain, side, switch):# packages the states of the oscillators into a velocity command for the robot
	current = limb.joint_angles()# get the angles at each joint
	vels = limb.joint_velocities()# get the velocities
	print osc
	if switch == 1:
		diff = gain*np.array([(osc[0]-(current[side+'_s0'])), (osc[1]-(current[side+'_s1'])), (osc[2]-(current[side+'_e0'])), (osc[3]-(current[side+'_e1'])),
		0, 0, 0]) - vgain*np.array([vels[side+'_s0'], vels[side+'_s1'], vels[side+'_e0'], vels[side+'_e1'], 0, 0, 0])# follow oscillator by calculating the difference with the joints current positions and transforming that into a velocity command#
	else:
		diff = gain*np.array([osc[0], osc[1], 1*osc[2], osc[3], 0, 0, 0]) - vgain*np.array([vels[side+'_s0'], vels[side+'_s1'], vels[side+'_e0'], vels[side+'_e1'], 0, 0, 0])# this has been altered to take spherical error values directly, bypassing the dynamics

	labels = [side+'_s0', side+'_s1', side+'_e0', side+'_e1', side+'_w0', side+'_w1', side+'_w2']# labels for each joint
	comm = {labels[0]:diff[0], labels[1]:diff[1], labels[2]:diff[2], labels[3]:diff[3], labels[4]:diff[4], labels[5]:diff[5], labels[6]:diff[6]}

	if any(t > 0.8 for t in diff):# going way too fast!! don't let the robot crash
		print("TOO FAST")
		sys.exit()# bug out
	return comm
	
def main():

	global CARTESIAN

	global S_Const
	global I_Const
	global downSample
	global depth
#-----------------------------------Startup Stuff-----------------------------------------------------------------
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                  	description=main.__doc__)
	required = parser.add_argument_group('required arguments')# this section is adding the required arguments
	parser.add_argument(
    	"-K", "--coupling", type=float, default=1.0,# this one is a float, but the next piece of code checks its value is within bounds
    	help=("Strength of the master/slave coupling")
	)
	args = parser.parse_args(rospy.myargv()[1:])

	rospy.init_node('slave_unit')# init node


	Left = baxter_interface.Limb('left')# arm interfaces
	Right = baxter_interface.Limb('right')
	Ktrue = args.coupling
	
	slave_targ = armslave(np.zeros(3), 0.1, Ktrue, Left, 'left', 1, S_Const, I_Const, 0.02)# create the slave oscillator(s)# CHECK SCALING
	initer = Left.joint_angles()# initial joint angles, preserved from before the robot moves
#-----------------------------------------------------------------------------------------------------------------


	dCount = 0# downsampling counter

	Slave_pub = rospy.Publisher('Slave_state', Pos, queue_size=1)# here, I think some delay is, if not beneficial, demonstrative of AS
	slave_buff = buff(0)

	Couple_pub = rospy.Publisher('Couple', Pos, queue_size=1)# publisher for coupling term
	M_pub = rospy.Publisher('M', Pos, queue_size=1)

	Master_sub = rospy.Subscriber('Master_state', Pos, slave_targ.add_master, queue_size = 10)# these lines are designed to update the coupling

	rospy.on_shutdown(slave_targ.shut)

	off = Left.endpoint_pose()
	
	integ = np.zeros(3)

	setup_time = 200

	r = rospy.Rate(100)
	for q in range(1, 50):# pause before beginning
		r.sleep()

	for n in range(1, setup_time):# try and eliminate offset by feeding directly into a controller, giving end effectors that have the same detected corrdinates (if not actual physical locations)
		left_pose = Left.endpoint_pose()# retrieve state of the body (left arm)
		right_pose = Right.endpoint_pose()# retrieve state of the body (right arm)
		
		right_angles = Right.joint_angles()# get the right arm's joint angles
		
		Sright = slave_targ.polarize(np.array([right_pose['position'].x, -1*right_pose['position'].y, right_pose['position'].z]))# spherical pose (right)
		Sleft = slave_targ.polarize(np.array([left_pose['position'].x, left_pose['position'].y, left_pose['position'].z]))# spherical pose (left)
		
		com = np.dot(slave_targ.mix, Sright-Sleft)


		#command = comZip(Left,com,1,1,'left',0)# format the velocity command
		#Left.set_joint_velocities(comZip(Left,np.zeros(depth),1,1,'left', off, com,0))# move the arm
		

		slave_targ.update(left_pose, right_angles, 'right', initer, Left.joint_angles(),1, np.array([left_pose['position'].x-off['position'].x, left_pose['position'].y-off['position'].y, left_pose['position'].z-off['position'].z]))# offset taken out to remove huge disparity between master and slave
		print 'not yet'
		r.sleep()

	origin_patch = left_pose['position'].y
	rec = 0
	while not rospy.is_shutdown():# main program loop
		
		left_pose = Left.endpoint_pose()# retrieve state of the body (combined output of oscillators)
		right_angles = Right.joint_angles()# do the same for the master arm

		com, smoothed, S, conv, inter = slave_targ.update(left_pose, right_angles, 'right', initer, Left.joint_angles(),1, np.array([left_pose['position'].x, left_pose['position'].y, left_pose['position'].z]))# update the state of the oscillators


		#Slave_pub.publish(Pos(com[0], com[1], com[2]))# publish the state of the slave target

		Slave_pub.publish(Pos(slave_targ.Cstate[0], slave_targ.Cstate[1], slave_targ.Cstate[2]))
		#Slave_pub.publish(Pos(slave_targ.multiState[0,0], slave_targ.multiState[0,1], slave_targ.multiState[0,2]))# publish the state of the slave target

		#C = np.array(c[0])
		#Couple_pub.publish(Pos(Cc[0,0], Cc[0,1], Cc[0,2]))# output difference between master and slave

		#M_pub.publish(Pos(M[0], M[1], M[2]))# publish the internal representation of the master (in polar coordinates)

		Sright = slave_targ.polarize(np.array([slave_targ.mast[0], slave_targ.mast[1], slave_targ.mast[2]]))# spherical pose (right)
		Sleft = slave_targ.polarize(np.array([left_pose['position'].x, left_pose['position'].y, left_pose['position'].z]))# spherical pose (left)

		ra = Sleft[0]
		p = Sleft[1]
		a = Sleft[2]
		Jacobian = [[np.cos(a)*np.sin(p), np.sin(a)*np.sin(p), np.cos(p)],[-np.sin(a)/(ra*np.sin(p)), np.cos(a)/(ra*np.sin(p)), 0],[(np.cos(a)*np.cos(p))/ra, (np.sin(a)*np.cos(p))/ra, -np.sin(p)/ra]]

		Vels = Left.endpoint_velocity()
		cartVel = np.array([Vels['linear'].x, Vels['linear'].y, Vels['linear'].z])
		V = np.dot(Jacobian,cartVel)
		SVel = np.array([V[0],V[2],V[1]])
		
		conv = np.dot(slave_targ.mix, Sright-Sleft-0*SVel)
		
		if (CARTESIAN == 0):# if not using Cartesian setpoint, set joint velocities
			Left.set_joint_velocities(comZip(Left, com, 50, 40, 'left', off, conv, 1))

		if (dCount >= (downSample-1)):# check if loop is modulo(downsample)
			dCount = 0
			#Left.set_joint_velocities(comZip(Left, com, 10, 0, 'left', off, conv, 1))#+np.min([10,rec/1000]) #np.min([1,rec/1000])
		else:
			dCount += 1# count up

		inter = slave_targ.mast - np.array([left_pose['position'].x, left_pose['position'].y, left_pose['position'].z])
		Couple_pub.publish(Pos(inter[0],inter[1],inter[2]))
		integ = integ+smoothed# accumulate coupling term

		r.sleep()
	return 0

if __name__ == '__main__':
    sys.exit(main())
