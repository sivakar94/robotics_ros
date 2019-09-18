#!/usr/bin/env python
# LORENZO JAMONE

# This code creates a ROS node that generates the coefficients of a cubic polynomial trajectory, starting from the movement parameters.

from AR_week5_test.srv import *
from AR_week5_test.msg import *
import rospy
import numpy.linalg as nl 
import numpy as np


# COMPUTES COEFFICIENTS (cubic_traj_coeffs) from PARAMETERS (cubic_traj_params)
def computeCoeffs(req):

    A = np.zeros( (4,4) ) 

# vector of desired initial and final position and velocity 
    pp = np.array ( [req.IN.p0, req.IN.v0, req.IN.pf, req.IN.vf] ) 

    p = pp.transpose()

#desired initial and final time of the trajectory    
    t0 = req.IN.t0
    tf = req.IN.tf

    
    A[0][0] = 1
    A[0][1] = t0
    A[0][2] = np.power(t0,2)
    A[0][3] = np.power(t0,3)
    A[1][0] = 0
    A[1][1] = 1
    A[1][2] = 2*t0
    A[1][3] = 3*np.power(t0,2)
    A[2][0] = 1
    A[2][1] = tf
    A[2][2] = np.power(tf,2)
    A[2][3] = np.power(tf,3)
    A[3][0] = 0
    A[3][1] = 1
    A[3][2] = 2*tf
    A[3][3] = 3*np.power(tf,2)

    tmp = np.dot(nl.inv(A),p) #solution of a system of linear equations

    c = cubic_traj_coeffs()

# coefficients of the cubic polynomial trajectory in the form y(x) = a0 + a1*x + a2*x^2 + a3*x^3

    c.a0 = tmp[0]
    c.a1 = tmp[1]
    c.a2 = tmp[2]
    c.a3 = tmp[3]
    c.t0 = t0
    c.tf = tf
    
    #test print
    print "Computed [%f ; %f ; %f ; %f]"%(c.a0 , c.a1 , c.a2 , c.a3 )

    return compute_cubic_trajResponse(c);

def computeCoeffs_server():
    rospy.init_node('compute_cubic_traj_server')
#ROS service made available
    s = rospy.Service('compute_cubic_traj', compute_cubic_traj, computeCoeffs) 
    print "Ready to compute coefficients of Cubic Polynomial Trajectory"
    rospy.spin()

if __name__ == "__main__":
    computeCoeffs_server()
