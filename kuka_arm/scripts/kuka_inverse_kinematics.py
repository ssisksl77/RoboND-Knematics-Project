#!/usr/bin/env python

from sympy import symbols, pi, sin, cos, atan2, sqrt, simplify
from sympy.matrices import Matrix
import tf

def build_mod_dh_matrix(s, theta, alpha, d, a):
    T = Matrix([[    cos(theta), -sin(theta), 0, a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(theta), -sin(alpha)*d],
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                [0,0,0,1]])
    # Substitue in the DH parameters into matrix
    T = T.sub(s)
    return T


def rot_x(q):
    return Matrix([[1, 0, 0],
                   [0, cos(q), -sin(q)],
                   [0, sin(q),  cos(q)]])

def rot_y(q):
    return Matrix([[cos(q), 0, sin(q)],
                   [     0, 1,      0],
                   [-sin(q),0, cos(q)]])

def rot_z(q):
    return Matrix([[cos(q), -sin(q), 0],
                   [sin(q),  cos(q), 0],
                   [     0,       0, 1]])

# Convertion factors between radians and degrees
dtr = pi/180
rtd = 180/pi 

# define dh parameter symbols
theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8') 
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

Modified DH parameters for kuka kr210
S = {alpha0:     0, d1:  0.75, a0:      0,
     alpha1: -pi/2, d2:     0, a1:   0.35, theta2: theta2 - pi/2,
     alpha2:     0, d3:     0, a2:   1.25,
     alpha3: -pi/2, d4:  1.50, a3: -0.054,
     alpha4:  pi/2, d5:     0, a4:      0,
     alpha5: -pi/2, d6:     0, a5:      0,
     alpha6:     0, d7: 0.303, a6:      0, theta7: 0}

def cal_123(R_EE, px, py, pz, roll, pitch, yaw):
    Rot = rot_z(180*dtr) * rot_y(-90*dtr)
    # print R_EE
    # Matrix([[0,0,-1],
    #         [0,-1,0],
    #         [1,0, 0]]
    R_EE = R_EE * Rot
    # print R_EE = 
    # Matrix([[r13, -r12, r11],
    #         [r23, -r22, r21],
    #         [r33, -r32, r31]])

    

