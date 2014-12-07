#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion, compose_matrix
from geometry_msgs.msg import Twist, Pose, PoseStamped,Point,Quaternion,TransformStamped
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from baxter_core_msgs.msg import EndpointState 
import baxter_interface
from baxter_interface import CHECK_VERSION
import time

import numpy

#From baxter_example ik service
import argparse 
import struct
import sys

from std_msgs.msg import Header

class BaxterForce:
    def __init__(self):
        print 'initializing'
        
        #Find the baxter from the parameter server
        self.baxter = URDF.from_parameter_server() 
        #Note:  A node that initializes and runs the baxter has to be running in the background for 
        #       from_parameter_server to to find the parameter.
        #       Older versions of URDF label the function "load_from_parameter_server"


        #Subscribe to the "baxter1_joint_states" topic, which provides the joint states measured by the baxter in a
        # ROS message of the type sensor_msgs/JointState. Every time a message is published to that topic run the 
        #callback function self.get_joint_states, which is defined below.
        
        self.end_eff_state_sub = rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,self.get_end_eff_state)
        
        self.listener=tf.TransformListener()
        self.initialize_gripper()
        # self.timer1 = rospy.Timer(rospy.Duration(0.01),) 

        return  

    def get_end_eff_state(self,data):
        
        try:
            self.end_eff_state=data.pose
            # print self.end_eff_state
        except  rospy.ROSInterruptException:
            self.end_eff_state = None
            pass

        return

    def initialize_gripper(self):
        # initialize interfaces
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)

        left.calibrate()
        # left.close()


def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('baxter_force')

    try:
        demo = BaxterForce()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__=='__main__':
    main()