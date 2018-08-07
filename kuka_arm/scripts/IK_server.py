#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

        # Twist Angles
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        # Link Lengths
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        # Link Offsets
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        # Joint Angles
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        dh_table = {
            alpha0:        0, a0:      0, d1: 0.75, q1: q1,
            alpha1: -pi / 2., a1:   0.35, d2:    0, q2: q2 - (pi / 2.),
            alpha2:       0., a2:   1.25, d3:    0, q3: q3,
            alpha3: -pi / 2., a3: -0.054, d4:  1.5, q4: q4,
            alpha4:  pi / 2., a4:      0, d5:    0, q5: q5,
            alpha5: -pi / 2., a5:      0, d6:    0, q6: q6,
            alpha6:        0, a6:      0, d7:0.303, q7: 0,  
        }

        def tf_matrix(alpha, a, d, theta):
            TF = Matrix([
                [             cos(theta),             -sin(theta),           0,               a],
                [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                [sin(theta) * sin(alpha), cos(theta) * sin(alpha),  cos(alpha),  cos(alpha) * d],
                [                      0,                       0,           0,               1]
                ])
            return TF

        t0_1  = tf_matrix(alpha0, a0, d1, q1).subs(dh_table)
        t1_2  = tf_matrix(alpha1, a1, d2, q2).subs(dh_table)
        t2_3  = tf_matrix(alpha2, a2, d3, q3).subs(dh_table)
        t3_4  = tf_matrix(alpha3, a3, d4, q4).subs(dh_table)
        t4_5  = tf_matrix(alpha4, a4, d5, q5).subs(dh_table)
        t5_6  = tf_matrix(alpha5, a5, d6, q6).subs(dh_table)
        t6_ee = tf_matrix(alpha6, a6, d7, q7).subs(dh_table)

        t0_ee = t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_ee 

        r, p, y = symbols('r p y')

        ROT_x = Matrix([
                [1,      0,       0],
                [0, cos(r), -sin(r)],
                [0, sin(r),  cos(r)]
                ])

        ROT_y = Matrix([
                [ cos(p), 0,  sin(p)],
                [      0, 1,       0],
                [-sin(p), 0,  cos(p)]
                ])

        ROT_z = Matrix([
                [cos(y), -sin(y), 0],
                [sin(y),  cos(y), 0],
                [     0,       0, 1]
                ])

        rot_ee = ROT_z * ROT_y * ROT_x

        Rot_Corr = ROT_z.subs(y, radians(180))*ROT_y.subs(p, radians(-90))

        rot_ee = rot_ee * Rot_Corr

        r0_3 = t0_1[0:3, 0:3] * t1_2[0:3, 0:3] * t2_3[0:3, 0:3]

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###


            rot_ee = rot_ee.subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([
                 [px],
                 [py],
                 [pz]
                 ])

            WC = EE - (0.303) * rot_ee[:,2]

            theta1 = atan2(WC[1], WC[0])

            l1_horiz = dh_table[a1]
            l1_z = dh_table[d1]

            wc_x = WC[0]
            wc_y = WC[1]
            wc_horiz = sqrt(wc_x * wc_x + wc_y * wc_y) - l1_horiz
            wc_z = WC[2] - l1_z

            l2_len = dh_table[a2] # 1.25
            l3_x = dh_table[d4] # 1.5
            l3_y = dh_table[a3] #-0.054
            l3_len = sqrt(l3_x * l3_x + l3_y * l3_y)
            l3_theta_offset = atan2(l3_y, l3_x)

            A = l3_len
            B = sqrt(wc_horiz * wc_horiz + wc_z * wc_z)
            C = l2_len

            a = acos((B * B + C * C - A * A) / (2 * B * C))
            b = acos((A * A + C * C - B * B) / (2 * A * C))
            c = acos((A * A + B * B - C * C) / (2 * A * B))
            d = atan2(wc_z, wc_horiz)
    
            theta2 = (pi / 2) - (a + d)
            theta3 = (pi / 2) - (b + l3_theta_offset)

        
            r0_3 = r0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            r3_6 = r0_3.inv("LU") * rot_ee

            theta4 = atan2(r3_6[2, 2], -r3_6[0, 2])
            theta5 = atan2(sqrt(r3_6[0,2]*r3_6[0,2] + r3_6[2,2]*r3_6[2,2]), r3_6[1,2])
            theta6 = atan2(-r3_6[1,1], r3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
