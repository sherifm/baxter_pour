#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion, compose_matrix
from geometry_msgs.msg import Twist, Pose, PoseStamped,Point,Quaternion,TransformStamped
from sensor_msgs.msg import JointState,Range
from urdf_parser_py.urdf import URDF
from baxter_core_msgs.msg import EndpointState 
from baxter_interface import CHECK_VERSION
import baxter_interface
import time

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
        #Note:  A node that initializes and runs the baxter has to be running in the background for 
        #       from_parameter_server to to find the parameter.
        #       Older versions of URDF label the function "load_from_parameter_server"


        #Subscribe to the "baxter1_joint_states" topic, which provides the joint states measured by the baxter in a
        # ROS message of the type sensor_msgs/JointState. Every time a message is published to that topic run the 
        #callback function self.get_joint_states, which is defined below.
        self.joint_state_sub = rospy.Subscriber("/robot/joint_states", JointState, self.get_joint_states)
        self.end_eff_state_sub = rospy.Subscriber("/robot/limb/left/endpoint_state",EndpointState,self.get_end_eff_state)
        self.left_hand_range= rospy.Subscriber("/robot/range/left_hand_range/state",Range,self.get_depth)
        
        self.listener=tf.TransformListener()
        # self.timer1 = rospy.Timer(rospy.Duration(0.01),) 

        #calibrate the gripper
        self.initialize_gripper()

        self.main()
        return  

    def initialize_gripper(self):
        # initialize interfaces
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        self.left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self.right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

        self.left_gripper.calibrate()

    def main(self):

        #define offset
        self.offset = .1

        #define your limb
        self.arm = 'left'

        #get kinect data
        self.kinect_data = numpy.array([-.03,-.09,0.77-self.offset,1])
        self.approach_pose = self.kinect_to_base(self.kinect_data)
        self.first_configuration = self.ik(self.arm, self.approach_pose)

        #move to first location
        self.cmd_joint_angles(self.first_configuration)

        #while we're not close to our object, keep making small adjustments
        self.approach_object(self.approach_pose)


    def approach_object(self,initial_pose):
        #open the gripper
        self.open_gripper()


        self.desired_depth = 0.06
        self.increment_value = 0.03

        temp = initial_pose
        count = 0

        time.sleep(2)
        print 'sleeping for 2 seconds'
        while self.depth > self.desired_depth:
            temp[0] = temp[0] + self.depth - self.desired_depth
            temp_configuration = self.ik(self.arm, temp)
            self.cmd_joint_angles(temp_configuration)
            print 'depth is',self.depth
            count += 1
            if count > 5:
                break
        if self.depth > self.desired_depth:
            print "object is close enough to grab!"
            #close the gripper
            self.close_gripper()
        pass

    def close_gripper(self):
        self.left_gripper.close()
        pass

    def open_gripper(self):
        self.left_gripper.open()
        pass



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

    def get_depth(self,data):
        self.depth=data.range

    # def ik_main(self):
    #     """RSDK Inverse Kinematics Example

    #     A simple example of using the Rethink Inverse Kinematics
    #     Service which returns the joint angles and validity for
    #     a requested Cartesian Pose.

    #     Run this example, passing the *limb* to test, and the
    #     example will call the Service with a sample Cartesian
    #     Pose, pre-defined in the example code, printing the
    #     response of whether a valid joint solution was found,
    #     and if so, the corresponding joint angles.
    #     """

    #     # self.parser.add_argument(
    #     #     '-l', '--limb', choices=['left', 'right'], required=True,
    #     #     help="the limb to test"
    #     # )
    #     # self.args = self.parser.parse_args(rospy.myargv()[1:])


    #     return ik(self.arm)

    def kinect_to_base(self,kinect_pose):
        try:
            time.sleep(1)
            (self.transl,self.quat)=self.listener.lookupTransform('base','kinect',rospy.Time(0))
            self.rot = euler_from_quaternion(self.quat)
            self.tf_SE3 = compose_matrix(angles=self.rot,translate = self.transl)
            base_pos=numpy.dot(self.tf_SE3,kinect_pose)
            print self.tf_SE3
            # print self.cmd_pos
        except (tf.Exception):
            rospy.logerr("Could not transform from "\
                         "{0:s} to {1:s}".format(base,kinect))
            pass
        return base_pos


    def ik(self,limb, cmd_pos):
        self.arg_fmt = argparse.RawDescriptionHelpFormatter
        self.parser = argparse.ArgumentParser(formatter_class=self.arg_fmt,
                                         description=main.__doc__)

        # t = self.listener.getLatestCommonTime('base', 'kinect') Better solution. Returns 0. Lear proper way: posestamped, frame_ID etc 
        # try:
        #     time.sleep(1)
        #     (self.transl,self.quat)=self.listener.lookupTransform('base','kinect',rospy.Time(0))
        #     self.rot = euler_from_quaternion(self.quat)
        #     self.tf_SE3 = compose_matrix(angles=self.rot,translate = self.transl)
        #     self.cmd_pos=numpy.dot(self.tf_SE3,kinect)
        #     print self.tf_SE3
        #     # print self.cmd_pos
        # except (tf.Exception):
        #     rospy.logerr("Could not transform from "\
        #                  "{0:s} to {1:s}".format(base,kinect))
        #     return
        
        self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
        self.ikreq = SolvePositionIKRequest()
        self.hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        self.x=cmd_pos[0]
        self.poses = {
            'left': PoseStamped(
                header=self.hdr,
                pose=Pose(
                    position=Point(
                        x=cmd_pos[0],
                        y=cmd_pos[1],
                        z=-cmd_pos[2]
                    ),
                    orientation=Quaternion(

                        x=.70711,
                        y=0,
                        z=.70711,
                        w=0,                     
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

            # print self.limb_joints

        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        return self.limb_joints



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
