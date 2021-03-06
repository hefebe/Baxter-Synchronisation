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


slave = [0, 0, 0]
mast = [0, 0, 0]
gain = 2

offset = [0.58, 0.3, 0.1]# this is the hardcoded 'untuck' pose for the left limb
Angles = [0.13, 1, -0.01, 0.03]# this is the orientation part
scaling = 0.05

# this takes a limb and a delta, which is added on top of the standard untucked pose (side handles the mirroring of the y coordinate)
def ik_test(limb, last, delta, scale, iksvc, ns, side):
    global offset# get global variables
    global Angles
    print(side)
    ikreq = SolvePositionIKRequest()# pretty self-explanatory, format a request for joint solution

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    sign = 1# defaults to left
    if side == "right":# if it's right, y is mirrored
	sign = -1

    new_Pose = {
        side: PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
			x=offset[0] + delta[0]*scale,# add the input from the oscillator to the current position (this could cause drift if messages are dropped)
			y=sign*offset[1] + sign*delta[1]*scale,
			z=offset[2] + delta[2]*scale,
    		),
                orientation=Quaternion(# orientation is currently preserved, which basically makes Baxter into a two-joint arm
			x = sign*Angles[0],# x is mirrored for right arm
			y = Angles[1],
			z = Angles[2],
			w = Angles[3],
		),
	    ),
        )
    }

    ikreq.pose_stamp.append(new_Pose[side])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:# throw error if service call times out (the program will crash)
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):# if the pose is achievable
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#---------------------------------------- Verbose Output ----------------------------------------------
        #print "\nIK Joint Solution:\n", limb_joints
        #print "------------------"
        #print "Response Message:\n", resp
	#limb.set_joint_positions(limb_joints)
#------------------------------------------------------------------------------------------------------
	return limb_joints# return solution
    else:
	#limb.set_joint_positions(last)
	print("fail")# no solution
        return last# bit of a kludge - failsafe to last value

def feedback(data):# callback for updating the slave target
	global slave
	slave[0] = data.x
	slave[1] = data.y
	slave[2] = data.z
	print slave

def Master(data):# callback for the master
	global mast
	mast[0] = data.x
	mast[1] = data.y
	mast[2] = data.z

def Differencing(current, desired, G, side):
	C = np.array([current[side+'_s0'], current[side+'_s1'], current[side+'_e0'], current[side+'_e1'], current[side+'_w0'], current[side+'_w1'], current[side+'_w2']])# turn current position to array
	D = np.array([desired[side+'_s0'], desired[side+'_s1'], desired[side+'_e0'], desired[side+'_e1'], desired[side+'_w0'], desired[side+'_w1'], desired[side+'_w2']])# turn desired position to array

	diff = G*(D-C)# get the difference value
	Adiff = abs(diff)# get absolute values

	if any(t > 1.5 for t in Adiff):# going way too fast!! don't let the robot crash
		print("TOO FAST")
		sys.exit()
		return {'left_s0':0, 'left_s1':0, 'left_e0':0, 'left_e1':0, 'left_w0':0, 'left_w1':0, 'left_w2':0}
	else:
		return {'left_s0':diff[0], 'left_s1':diff[1], 'left_e0':diff[2], 'left_e1':diff[3], 'left_w0':diff[4], 'left_w1':diff[5], 'left_w2':diff[6]}
	
def main():
	global scaling# get global scaling
	global mast# get the global master state
	global slave# ditto slave
	global gain# ditto gain
	global offset# ditto offset

	rospy.init_node('move_unit')# init node
	Left = baxter_interface.Limb('left')# Left Arm!!
	Right = baxter_interface.Limb('right')# Right Arm!!

	Feedback_sub = rospy.Subscriber('Slave_state', Pos, feedback, queue_size = 1)# I want to discard old values

	Mast_sub = rospy.Subscriber('Master_state', Pos, Master, queue_size = 1)

	NS = "ExternalTools/" + "left" + "/PositionKinematicsNode/IKService"# create a persistant service
        serv = rospy.ServiceProxy(NS, SolvePositionIK, 'TRUE')#               (better for continuous use)

	NS2 = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"# one for the right arm
        serv2 = rospy.ServiceProxy(NS2, SolvePositionIK, 'TRUE')

	#first_angles = Left.joint_angles()# save the initial position
	#first_angles2 = Right.joint_angles()
	init_pose = Left.endpoint_pose()
	offset = np.array([init_pose['position'].x, init_pose['position'].y, init_pose['position'].z])#


	initRight = {'right_s0':-0.5, 'right_s1':-1.2, 'right_e0':-0.3, 'right_e1':0, 'right_w0':0, 'right_w1':0.9, 'right_w2':0.36}#[-0.5, -1.2, 0.3, 0]
	initLeft = {'left_s0':0.5, 'left_s1':-1.2, 'left_e0':0.3, 'left_e1':0, 'left_w0':0, 'left_w1':0.9, 'left_w2':0.36}

	r = rospy.Rate(50)
	for n in range(1, 400):# for some reason this must run in a loop
		#poser = ik_test(Left, Left.joint_angles(), [0, 0, 0], scaling, serv, NS, 'left')
		#Left.set_joint_positions(initLeft)
		#poser = ik_test(Right, Right.joint_angles(), [0, 0, 0], scaling, serv2, NS2, 'right')
		#Right.set_joint_positions(initRight)
		r.sleep()

	while not rospy.is_shutdown():
		poser = ik_test(Left, Left.joint_angles(), slave, 1, serv, NS, 'left')# get left pose using master
		#diff = Differencing(Left.joint_angles(), poser, gain, 'left')# get the difference
		
		Left.set_joint_positions(poser)
		#Left.set_joint_velocities(diff)# feed this into joint velocities
		
		#poser2 = ik_test(Right, Right.joint_angles(), mast, scaling, serv2, NS2, 'right') #the mast signal does not trigger this: why?
		#diff2 = Differencing(Right.joint_angles(), poser2, gain, 'right')# get the difference
		#Right.set_joint_positions(poser2) # do not move this if using arm2arm
		#Right.set_joint_velocities(diff2)# feed this into joint velocities
		
		
		#print(Left.joint_velocities())
		r.sleep()
	serv.close()
	return 0

if __name__ == '__main__':
    sys.exit(main())
