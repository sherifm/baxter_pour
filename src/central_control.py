#! /usr/bin/env python

# import roslib; roslib.load_manifest('square_action')
import rospy
import numpy
import tf

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import baxter_pour.msg
import time




import math
from tf.transformations import euler_from_quaternion, compose_matrix
from geometry_msgs.msg import Twist, Pose, PoseStamped,Point,Quaternion,TransformStamped
from sensor_msgs.msg import JointState,Range
from baxter_core_msgs.msg import EndpointState 
from baxter_interface import CHECK_VERSION
import baxter_interface
#From baxter_example ik service
import argparse 
import struct
import sys
from std_msgs.msg import Header
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import roslib
import actionlib
import baxter_pour.msg




class central_control:

    def __init__ (self):
        self.listener=tf.TransformListener()
        self.main()


    def main(self):

        #get kinect data
        # self.kinect_data = numpy.array([.05,.279,.762,1])
        # self.pickup_pose = self.kinect_to_base(self.kinect_data)

        #weigih each object. And store into unsorted matrix
        # weigh_object = self.weigh_client()
        # print "Result:", weigh_object.weight, "Kg"


        #create and sort data matrix

        self.sorted_matrix = ((10,['ID1',.6,.4,-.02]),(1,['ID2',.6,.6,-.02]),(100,['ID3',.6,.5,-.02]))
        self.move_to_sort(self.sorted_matrix)
        #self.pickup_pose = numpy.array([0.6 ,0.4 , -0.0232258427437])
        #self.dropoff_pose = numpy.array([0.9 ,0.4 , -0.0232258427437])
        #move_object = self.move_client(self.pickup_pose,self.dropoff_pose)




    def move_to_sort(self,matrix):
        #self.dropoff_pose = numpy.array([0.9 ,0.4 , -0.0232258427437])


        self.pickup_pose = numpy.array([matrix[0][1][1] ,matrix[0][1][2] , matrix[0][1][3]])
        self.dropoff_pose = numpy.array([ .7,.6 ,-.02])
        
        self.move_client(self.pickup_pose,self.dropoff_pose)

        self.pickup_pose = numpy.array([matrix[1][1][1] ,matrix[1][1][2] , matrix[1][1][3]])
        self.dropoff_pose = numpy.array([ .7,.5 ,-.02])
        
        self.move_client(self.pickup_pose,self.dropoff_pose)

        self.pickup_pose = numpy.array([matrix[2][1][1] ,matrix[2][1][2] , matrix[2][1][3]])
        self.dropoff_pose = numpy.array([ .7,.4 ,-.02])
        
        self.move_client(self.pickup_pose,self.dropoff_pose)



        #unsorted_matrix = ((10,[ID1,x1,y1,z1]),(1,[ID2,x2,y2,z2]),(100,[ID3,x3,y3,z3]))
        #sorted_matrix = sorted(unsorted_matrix,key=lambda dist:dist[0])


    def weigh_client(name):
        
        # Creates the SimpleActionClient, passing the type of the action
        # (MoveAction) to the constructor.
        client = actionlib.SimpleActionClient('weigh_server', baxter_pour.msg.WeighAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = baxter_pour.msg.WeighGoal(x_pickup=0.856 , y_pickup=0.58 , z_pickup=-0.097)

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Gets the result of executing the action

        return client.get_result() 

    def move_client(name,pickup_pose,dropoff_pose):
        print 'calling client'
        # Creates the SimpleActionClient, passing the type of the action
        # (MoveAction) to the constructor.
        client = actionlib.SimpleActionClient('move_server', baxter_pour.msg.MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = baxter_pour.msg.MoveGoal(x_pickup=pickup_pose[0] , y_pickup=pickup_pose[1] , z_pickup=pickup_pose[2],x_dropoff=dropoff_pose[0],
            y_dropoff=dropoff_pose[1],z_dropoff=dropoff_pose[2])

        #These are empirically derived values for a good reach of pick-up and place maximums
        #(x_pickup=0.83 , y_pickup=-0.05177498224766 , z_pickup=-0.06805627181,x_dropoff=0.83,
            # y_dropoff=0.6,z_dropoff=-0.06805627181)

        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Gets the result of executing the action
        return client.get_result() 


    def kinect_to_base(self,kinect_pose):
        try:
            time.sleep(1)
            (self.transl,self.quat)=self.listener.lookupTransform('base','kinect',rospy.Time(0))
            self.rot = euler_from_quaternion(self.quat)
            self.tf_SE3 = compose_matrix(angles=self.rot,translate = self.transl)
            print self.tf_SE3
            base_pos=numpy.dot(self.tf_SE3,kinect_pose)
            print base_pos
            # print self.cmd_pos
        except (tf.Exception):
            rospy.logerr("Could not transform from "\
                         "{0:s} to {1:s}".format(base,kinect))
            pass
        return base_pos


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('central_control')
        central_control()
    
    except rospy.ROSInterruptException:
        print "program interrupted before completion"


