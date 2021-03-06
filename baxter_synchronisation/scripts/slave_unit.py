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
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

#from baxter_pykdl import baxter_kinematics


depth = 1
S_Const = 10

class buff:# buffers values and performs noise filtering
	def __init__(self, length):
		self.length = length
		self.wait = np.zeros((3,self.length))

	def filterise(self, ePoint):
		self.wait[:, 1:self.length-1] = self.wait[:, 0:self.length-2]# shuffle up
		self.wait[:,0] = np.array([ePoint['position'].x, ePoint['position'].y, ePoint['position'].z])# add newest member
		avg = np.sum(self.wait,1)# mean value
		return avg/self.length# return filtered value
	
class slave:
	def __init__(self, start, speed, kay, begin_pose, scaling=0.05):# takes initial conditions and time constant

		global depth
		
		self.state = start# state in x, y and z
		self.mast = np.zeros(3)# state of the coupling
		self.feedback = np.zeros(3)# feedback coming back from the plant
		self.power = 0# final model defaults to zero, although master feedback could force this
		
		self.state = np.array(start)# set the state

		self.multiState = np.zeros((3,depth))
		self.multiFeed = np.zeros((3,depth))
		
		self.rate = speed# set the rate

		self.DC = np.matrix(np.zeros(3))

		self.mrate = np.linspace(speed/depth, speed, depth)# gives a range of frequencies
		self.mK = np.linspace(kay, kay, depth)#kay + kay/depth -

		self.mix = np.ones((depth,1))

		self.initial = begin_pose
		
		# self.a = 0.15# Rossler constants
		# self.b = 0.2
		# self.c = 10
		
		self.K = kay# this is the coupling constant

		self.scaling = scaling

		self.offset = np.array([0.58, 0.3, 0.1])
		
		
	def add_master(self, data):# this updates the coupling - no particular reason to have this trigger on master, rather than slave, but we'll see which is better
		#if data.x:# this just prevents an edge case where the coupling is too large
		self.mast[0] = data.x
		
		

		self.mast[1] = data.y

		
		
		self.mast[2] = data.z
		

		print("yes")
	
	def update(self, poser, internal, ext = [0,0,0]):
		global depth
		# the coupling subtracts the initial offset from the robotic feedback
		#self.rate*(self.power-self.feedback[0]) + self.K*(self.mast[0] - (poser['position'].x))
		# self.K*(self.scaling*self.mast[0] - (poser['position'].x-self.offset[0]))# original coupling
		if 1:#any (C !=0 for C in self.mast)
			#self.state = np.array(self.state) +self.rate*(self.power-np.array(self.feedback))+ self.K*(self.scaling*np.array(self.mast) - (poser['position'].x-self.offset))

			#self.feedback = self.feedback + self.rate*(self.state)-self.K*(self.scaling*self.mast - (poser['position'].x-self.offset))# feedback term in the oscillator
			
			
			if internal==1:
				S = self.state*self.scaling
			elif internal == 0:
				S = ext# get the state of another oscillator of the same type
			else:
				S = poser#-self.offset
				print self.state
			couple = np.dot(np.matrix(self.mK).T,np.matrix(np.array(self.mast)- S.T))# scaling taken out for arm2arm #self.scaling* 
			self.multiState = self.multiState + np.multiply(self.power-self.multiFeed, self.mrate)+ np.transpose(couple)
			self.multiFeed = self.multiFeed + np.multiply(self.multiState, self.mrate)- np.transpose(couple)

			self.DC = self.DC + (self.rate/25)*(self.mast-self.DC)
			self.state = np.dot(self.multiState,self.mix) + 0*self.DC.T

		else:
			#self.state = self.state +self.rate*(self.power-self.feedback)

			#self.feedback = self.feedback + self.rate*(self.state)# feedback term in the oscillator

			self.multiState = self.multiState + np.multiply(self.power-self.multiFeed, self.mrate)
			self.multiFeed = self.multiFeed + np.multiply(self.multiState, self.mrate)
			self.state = np.transpose(np.dot(self.multiState,self.mix))
			self.state = self.state[0]


#-----------------------------------------------------ROSSLER STUFF----------------------------------------------------
	# def update(data)
		# self.mast[1] = data.x
		# self.mast[2] = data.y
		# self.mast[3] = data.z
		
		# self.state[1] = self.state[1] + self.rate*(-self.state[2] -self.state[3])# calc X (too many selfs, I'll remove them when I'm sure it's safe)
		# self.state[2] = self.state[2] + self.rate*(self.state[1] + self.a*self.state[2])# calc Y
		# self.state[3] = self.state[3] + self.rate*((self.state[1] - self.c)*self.state[3] +self.b)# calc z
		# self.state[:] = self.state[:] + self.K*(self.mast[:]-self.mast[:])# perform coupling
#----------------------------------------------------------------------------------------------------------------------		
	
def main():
	global S_Const
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

	#arm = baxter_interface.Limb('left')

	Left = baxter_interface.Limb('left')# arm interfaces
	Right = baxter_interface.Limb('right')


	slave_targ = slave(np.zeros(3), 0.1, args.coupling, Left.endpoint_pose(),0.05)# create the slave oscillator
	Slave_buff = buff(S_Const)# buffer for slave values
#-----------------------------------------------------------------------------------------------------------------
	
	Slave_pub = rospy.Publisher('Slave_state', Pos, queue_size=1)# here, I think some delay is, if not beneficial, demonstrative of AS
	Arm_pub = rospy.Publisher('Slave_true', Pos, queue_size=1)# true position
	Master_sub = rospy.Subscriber('Master_state', Pos, slave_targ.add_master, queue_size = 1)# these lines are designed to update the coupling
	
	init_pose = Left.endpoint_pose()
	slave_targ.offset = np.array([init_pose['position'].x, -init_pose['position'].y, init_pose['position'].z])

	r = rospy.Rate(200)
	while not rospy.is_shutdown():
		
		left_pose = Left.endpoint_pose()# update the state of the slave target

		smooth = Slave_buff.filterise(left_pose)

		poz = np.array([left_pose['position'].x, left_pose['position'].y, left_pose['position'].z])#np.array([slave_filter.state[0]/20, slave_filter.state[1]/20, slave_filter.state[2]/20])



		slave_targ.update(smooth,2)

		

		Slave_pub.publish(Pos(slave_targ.state[0], slave_targ.state[1], slave_targ.state[2]))# publish the state of the slave target
		puz = Left.joint_angles()
		
		Arm_pub.publish(Pos(poz[0], poz[1], poz[2]))
		print('go')
		
		r.sleep()
	#serv.close()
	return 0

if __name__ == '__main__':
    sys.exit(main())
