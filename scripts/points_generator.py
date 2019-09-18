#!/usr/bin/env python
# LORENZO JAMONE

# This code creates a ROS node that publishes random parameters for a cubic polynomial trajectory.
# Parameters are:
# p0 initial position
# pf final position
# v0 initial velocity
# vf final velocity
# t0 starting time = 0
# tf final time

import numpy as np
import rospy
from AR_week5_test.msg import cubic_traj_params

# Frequency of publishing data
pubFreq = 0.05
# faster publishing frequency to be used for testing...
#pubFreq = 0.5    


#Minimum and Maximum values for the parameters
P_MAX = 10
P_MIN = -10
V_MAX = 10
V_MIN = -10
T_MIN = 5
T_MAX = 10

par = cubic_traj_params()


# FUNCTION WHICH GENERATES THE RANDOM DATA
def randGen(data):

	data.p0 = np.random.uniform(P_MIN,P_MAX) #random real number from P_MIN to P_MAX
	data.pf = np.random.uniform(P_MIN,P_MAX)
	data.v0 = np.random.uniform(V_MIN,V_MAX)
	data.vf = np.random.uniform(V_MIN,V_MAX)
	data.t0 = 0
	data.tf = np.random.uniform(T_MIN,T_MAX)


# FUNCTION WHICH GENERATES THE RANDOM DATA (for trajectories with initial and final velocities = 0)
# Could be useful in real applications
def randGenV0(data):

	data.p0 = np.random.uniform(P_MIN,P_MAX)
	data.pf = np.random.uniform(P_MIN,P_MAX)
	data.v0 = 0
	data.vf = 0
	data.t0 = 0
	data.tf = np.random.uniform(T_MIN,T_MAX)


# FUNCTION WHICH PUBLISHES THE DATA

def write():

    pub = rospy.Publisher('params', cubic_traj_params, queue_size=1)
    rospy.init_node('generator', anonymous=True)
    rate = rospy.Rate(pubFreq) # publishes at pubFreq hz
    
    while not rospy.is_shutdown():
        randGen(par)
        #randGenV0(par) #forTesting
        str = "%f " %par.p0 + "%f " %par.pf + "%f " %par.v0 + "%f " %par.vf + "%f " %par.t0 + "%f " %par.tf
        rospy.loginfo(str)
        pub.publish(par)
        rate.sleep() #sleeps for about 1/pubFreq seconds


# MAIN FUNCTION

if __name__ == '__main__':
    
    try:
        write()
    except rospy.ROSInterruptException:
        pass
