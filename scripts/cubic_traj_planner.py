#!/usr/bin/env python
# LORENZO JAMONE

# This code creates a ROS node that reads movement parameters for a cubic polynomial trajectory (from a ROS Topic), computes the related coefficients (by using a ROS Service), and then publishes the coefficients (on a ROS Topic).
# Coefficients are: a0, a1, a2, a3.
# Cubic Polynomial function: f(x) = a0 + a1*x + a2*x^2 + a3*x^3.

import rospy
from AR_week5_test.srv import *
from AR_week5_test.msg import *


#The callbackP function is executed when a message (with the movement parameters) is received. 
def callbackP(par):
    
    co = compTrajClient(par) #computes coefficients from parametes (calling the remote ROS Service)
    
    print co #test print

    write(co) #publishes the coefficients (on a ROS Topic)


#Reader NODE: as soon as a message is received, the callbackP function is executed. 
def reader():

    rospy.init_node('reader', anonymous=True)
    rospy.Subscriber("params", cubic_traj_params, callbackP)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#This calls the remote ROS Service
def compTrajClient(p):
    rospy.wait_for_service('compute_cubic_traj')
    try:
        compTraj = rospy.ServiceProxy('compute_cubic_traj', compute_cubic_traj)
        resp1 = compTraj(p)
        return resp1.OUT 
#takes the OUT component of response, as defined in the  compute_cubic_traj.srv file

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#This is the publisher 
def write(c):

    pub = rospy.Publisher('coeffs', cubic_traj_coeffs, queue_size=1)
    pub.publish(c)
    
    #test print
    #str = "%f " %c.a0 + "%f " %c.a1 + "%f " %c.a2 + "%f " %c.a3 + "%f " %c.t0 + "%f " %c.tf
    #rospy.loginfo(str)
    

if __name__ == '__main__':
    
    reader()
    	

