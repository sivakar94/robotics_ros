#!/usr/bin/env python
# LORENZO JAMONE

# This code creates a ROS node that reads coefficients of a cubic polynomial trajectory from a ROS Topic and creates points of the cubic polynomial trajectory to be sent on three separate ROS Topics: position trajectory, velocity trajectory, acceleration trajectory.


import rospy
from AR_week5_test.srv import *
from AR_week5_test.msg import *
from std_msgs.msg import Float32
import numpy as np
import time



def callbackC(coef):
    
    #test print
    print coef.tf
    
    write(coef)


#THE NODE WAITS FOR A MESSAGE (which contains the coefficients) AND THEN CALLS THE CALLBACK FUNCTION    
def reader():
    
    rospy.init_node('trajPointsGen', anonymous=True)  
    rospy.Subscriber("coeffs", cubic_traj_coeffs, callbackC)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


#GENERATION OF THE TRAJECTORY POINTS
def compP(t,c):

    return c.a0 + c.a1*t + c.a2*np.power(t,2) + c.a3*np.power(t,3)

def compV(t,c):

    return c.a1 + 2*c.a2*t + 3*c.a3*np.power(t,2)

def compA(t,c):

    return 2*c.a2 + 6*c.a3*t

#PUBLISHES THE TRAJECTORY POINTS, BASED ON THE COEFFICIENTS    
def write(co):

    pubP = rospy.Publisher('trajPos', Float32, queue_size=1)
    pubV = rospy.Publisher('trajVel', Float32, queue_size=1)
    pubA = rospy.Publisher('trajAcc', Float32, queue_size=1)

    steps = 100

#duration of the time steps
    dt = co.tf / steps
    #ratePub = rospy.Rate(1/dt) # cycles at 1/dt hz

    stTime = time.clock()
    t_i = 0
    
    #while (time.clock() - stTime) < co.tf:
    while t_i < co.tf:
    	str = "T: %.2f  " %t_i + "P: %.2f  " %compP(t_i,co) + "V: %.2f  " %compV(t_i,co) + "A: %.2f  " %compA(t_i,co)
    	rospy.loginfo(str)
    	pubP.publish(compP(t_i,co))
    	pubV.publish(compV(t_i,co))
    	pubA.publish(compA(t_i,co))
        t_i = t_i + dt #time instants on which the trajectory points are computed
    	time.sleep(dt)
      

if __name__ == '__main__':
    
    	reader()

