#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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


# STEP 0 util function
def get_hypotenuse(a, b):
    return sqrt(a**2 + b**2)


# STEP 0 util function
def get_law_of_cosine_angle(a, b, c):
    """
    value an angle against c(3rd parameter)
    """

    cos = (a**2 + b**2 - c**2)
    sin = sqrt(1 -cos**2)
    angle = atan2(sin, cos)

    return angle


# STEP 0 util function(DH-Matrix)
def dh_matrix(dh_params, theta, alpha, d, a):
    Tab = Matrix([
        [           cos(theta),           -sin(theta),           0,             a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [                    0,                     0,           0,             1]])
    Tab = Tab.subs(dh_params)
    return Tab


# STEP0 ROTATION X, Y, Z
def rot_x(q):
    return Matrix([[1,      0,       0],
                   [0, cos(q), -sin(q)],
                   [0, sin(q),  cos(q)]])


def rot_y(q):
    return Matrix([[  cos(q), 0, sin(q)],
                   [       0, 1,      0],
                   [ -sin(q), 0, cos(q)]])

def rot_z(q):
    return Matrix([[ cos(q), -sin(q), 0],
                   [ sin(q),  cos(q), 0],
                   [      0,       0, 1]])


# STEP 1 GET WC
# dg is fixed (default)
def get_wrist_center(gripper_point, R0g, dg=0.303):
    Gx, Gy, Gz = gripper
    G0g_x, G0g_y, G0g_z = R0g[0,2], R0g[1,2], R0g[2,2]

    WC_x = Gx - G0_x*dg
    WC_y = Gy - G0_y*dg
    WC_z = Gz - G0_z*dg

    return WC_x, WC_y, WC_z

# SET 2 get first 1,2,3 joints' angle
# using WC (resulft from 'get_wirst_center')
def get_first_three_angles(wrist_center):
    x, y, z = wrist_center
    # Now, we can get theta1 (joint1's angle)
    # theta1 = atan2(y, x)

    a1, a2, a3  = 0.35, 1.25, -0.054  # from dh
    d1, d4 = 0.75, 1.5
    L = get_hypotenuse(d4, a3)
    x_prime = get_hypotenuse(x, y)

    # let's get theta2 (joint2's angle)
    mx = x_prime -a1  #  wrist_center's x - link length
    mz = z - d1  # wrist_center's - link offset
    m  = get_hypotenuse(mx, mz)  # hypotenuse from joint2 WC(joint5)
    alpha = atan2(mz, mx)
    beta  = get_law_of_consine_angle(m, a2, L)
    # Now, we can get theta2(joint2's angle)
    # theta2 = 90degree - alpha - beta

    gamma = get_law_of_consine_angle(L, a2, m)  # get angle against m
    alpha_theta3 = atan2(d4, a3)
    # Now, we can get theta3(joint3's angle)
    # theta3 = -(gamma - alpha_theta3)

    theta1 = atan2(y, x)
    theta2 = pi/2 - alpha - beta
    theta3 = -(gamma - alpha_theta3)

    return theta1, theta2, theta3


# FINAL!!! GET LAST 3 joints(4,5,6) angles
# T36 = simplify(T34*T45*T56)
# get T36 and check this out
def get_last_three_angles(T36):
    joint4_sin  =  T36[2, 2]  # sin(theta4)*sin(theta5)
    joint4_cos  = -T36[0, 2]  # -1 * -(sin(theta5)*cos(theta4))
    theta4      = atan2(joint4_sin, joint4_cos)  # sin(theta4) / cos(theta4)

    joint5_sin = sqrt(T36[0,2]**2 + T36[2,2]**2)
    # sqrt(sin(theta4)^2*sin(theta5)^2 + sin(theta5)^2*cos(theta4)^2)
    # sqrt(sin(theta5)^2*(sin(theta4)^2 + cos(theta4)^2))
    # sqrt(sin(theta5)^2*(1))
    # sin(theta5)
    joint5_cos = T36[1, 2]  # cos(theta5)
    theta5     = atan2(joint5_sin, joint5_cos)  # sin(theta5) / cos(theta5)

    joint6_sin = -T36[1, 1]  # -1 * (-sin(theta5)*sin(theta6))
    joint6_cos = T36[1, 0]   # sin(theta5)*cos(theta6)
    theta6     = atan2(joint6_sin, joint6_cos)  # sin(theta6) / cos(theta6)

    return theta4.evalf(), theta5.evalf(), theta6.evalf()


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        # Here we go!!
        theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
	# Create Modified DH parameters
        dh_params = {
            alpha0:     0, d1:  0.75, a0:      0,
            alpha1: -pi/2, d2:     0, a1:   0.35, theta2: theta2 - pi/2,
            alpha2:     0, d3:     0, a2:   1.24,
            alpha3: -pi/2, d4:  1.50, a3: -0.054,
            alpha4:  pi/2, d5:     0, a4:      0,
            alpha5: -pi/2, d6:     0, a5:      0,
            alpha6:     0, d7: 0.303, a6:      0, theta7: 0
        }

	# Create individual transformation matrices
        T01 = dh_matrix(dh_params, theta1, alpha0, d1, a0)
        T12 = dh_matrix(dh_params, theta2, alpha1, d2, a1)
        T23 = dh_matrix(dh_params, theta3, alpha2, d3, a2)
        T34 = dh_matrix(dh_params, theta4, alpha3, d4, a3)
        T45 = dh_matrix(dh_params, theta5, alpha4, d5, a4)
        T56 = dh_matrix(dh_params, theta6, alpha5, d6, a5)

	# Extract rotation matrices from the transformation matrices
        T03 = simplify(T01*T12*T23)
        R03 = T03[:3,:3] # 4X4 -> 3X3
        R03T = R03.T

	###

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
            Rug  = (rot_z(pi)* rot_y(-pi/2)).T  # from URDF
            RugT = Rug.T
            R0g_value = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
            R0g_eval  = R0u_value * RugT  # R0g = R0u*Rug

            gripper_point = px, py, pz
            WC = get_wrist_center(gripper_point, R0g_eval)
            theta1, theta2, theta3 = get_first_three_angles(WC)
            R03T_eval = R03T.evalf(
                subs = {
                    theta1: theta1.evalf(),
                    theta2: theta2.evalf(),
                    theta3: theta3.evalf()
                })

            R36_eval = R03T_eval * R0g_eval
            theta4, theta5, theta6 = get_last_three_angles(R36_eval) 
            #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###
		
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
