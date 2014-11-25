#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose, PoseStamped,Point,Quaternion
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from baxter_core_msgs.msg import EndpointState 
import baxter_interface

import numpy

#From baxter_example ik service
import argparse
import struct
import sys

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class BaxterPour:
	def __init__(self):
		print 'initializing'
		
		#Find the baxter from the parameter server
		self.baxter = URDF.from_parameter_server() 
		#Note: 	A node that initializes and runs the baxter has to be running in the background for 
		#		from_parameter_server to to find the parameter.

		
		#		Older versions of URDF label the function "load_from_parameter_server"


		#Subscribe to the "baxter1_joint_states" topic, which provides the joint states measured by the baxter in a
		# ROS message of the type sensor_msgs/JointState. Every time a message is published to that topic run the 
		#callback function self.get_joint_states, which is defined below.
		self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.get_joint_states)
		self.end_eff_state_sub = rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,self.get_end_eff_state)
		
		self.cmd_pos=numpy.array([0.78,0.34,-0.07])


		# self.timer1 = rospy.Timer(rospy.Duration(0.01),) 
		self.ik_main()

		return	

	def get_joint_states(self,data):
		try:
			self.q_sensors = data.position
		except rospy.ROSInterruptException: 
			self.q_sensors = None
			pass
		return

	def get_end_eff_state(self,data):
		
		try:
			self.end_eff_state=data.pose
			# print self.end_eff_state
		except  rospy.ROSInterruptException:
			self.end_eff_state = None
			pass

		return

	def ik_main(self):
	    print "Hello"
	    """RSDK Inverse Kinematics Example

	    A simple example of using the Rethink Inverse Kinematics
	    Service which returns the joint angles and validity for
	    a requested Cartesian Pose.

	    Run this example, passing the *limb* to test, and the
	    example will call the Service with a sample Cartesian
	    Pose, pre-defined in the example code, printing the
	    response of whether a valid joint solution was found,
	    and if so, the corresponding joint angles.
	    """
	    self.arg_fmt = argparse.RawDescriptionHelpFormatter
	    self.parser = argparse.ArgumentParser(formatter_class=self.arg_fmt,
	                                     description=main.__doc__)
	    # self.parser.add_argument(
	    #     '-l', '--limb', choices=['left', 'right'], required=True,
	    #     help="the limb to test"
	    # )
	    # self.args = self.parser.parse_args(rospy.myargv()[1:])
	    self.arm = 'left'
	    return self.ik_test(self.arm)	

	def ik_test(self,limb):
	    
	    #rospy.init_node("rsdk_ik_service_client")
	    self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
	    self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
	    self.ikreq = SolvePositionIKRequest()
	    self.hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	    self.x=self.cmd_pos[0]
	    self.poses = {
	        'left': PoseStamped(
	            header=self.hdr,
	            pose=Pose(
	                position=Point(
	                    x=self.cmd_pos[0],
	                    y=self.cmd_pos[1],
	                    z=-self.cmd_pos[2]
	                ),
	                orientation=Quaternion(
	                    x=0.367048116303,
	                    y=0.885911751787,
	                    z=-0.108908281936,
	                    w=0.261868353356,
	                ),
	            ),
	        ),
	        'right': PoseStamped(
	            header=self.hdr,
	            pose=Pose(
	                position=Point(
	                    x=0.656982770038,
	                    y=-0.852598021641,
	                    z=0.0388609422173,
	                ),
	                orientation=Quaternion(
	                    x=0.367048116303,
	                    y=0.885911751787,
	                    z=-0.108908281936,
	                    w=0.261868353356,
	                ),
	            ),
	        ),
	    }

	    self.ikreq.pose_stamp.append(self.poses[limb])
	    try:
	        rospy.wait_for_service(self.ns, 5.0)
	        self.resp = self.iksvc(self.ikreq)
	    except (rospy.ServiceException, rospy.ROSException), e:
	        rospy.logerr("Service call failed: %s" % (e,))
	        return 1

	    # Check if result valid, and type of seed ultimately used to get solution
	    # convert rospy's string representation of uint8[]'s to int's
	    self.resp_seeds = struct.unpack('<%dB' % len(self.resp.result_type),
	                               self.resp.result_type)
	    if (self.resp_seeds[0] != self.resp.RESULT_INVALID):
	        self.seed_str = {
	                    self.ikreq.SEED_USER: 'User Provided Seed',
	                    self.ikreq.SEED_CURRENT: 'Current Joint Angles',
	                    self.ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
	                   }.get(self.resp_seeds[0], 'None')
	        # print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
	        #       (self.seed_str,))
	        # Format solution into Limb API-compatible dictionary
	        self.limb_joints = dict(zip(self.resp.joints[0].name, self.resp.joints[0].position))
	        #print "\nIK Joint Solution:\n", self.limb_joints
	        #print "------------------"
	        #print "Response Message:\n", self.resp

	        print self.limb_joints
	        return self.cmd_joint_angles(self.limb_joints)
	    else:
	        print("INVALID POSE - No Valid Joint Solution Found.")

	    return 0

	def cmd_joint_angles(self, data):
		self.limb = baxter_interface.Limb(self.arm)
		self.limb.move_to_joint_positions(data)
		print "i'm trying to move"
		pass


def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('baxter_pour')

    try:
        demo = BaxterPour()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__=='__main__':
    main()
