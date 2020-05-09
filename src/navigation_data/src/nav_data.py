#!/usr/bin/env python

import rospy
import csv
import numpy as np
from navigation_data.msg import NavigationEstimate
from std_msgs.msg import Header




def navCallback(data):
    time = data.header.stamp
    pose = data.pose.position
    heading = data.pose.orientation
    b = np.asarray([ [pose.x,pose.y,pose.z,time,0,0] ])

    with open('hei.csv','ab') as fd:
        np.savetxt(fd, b, delimiter=",", fmt="%s")

    ##NED = lla2ned()



def main():
    rospy.init_node('nav_data', anonymous=True)

    #Vil legge til ros::time() i filnavn

    rospy.Subscriber("NavigationEstimate", NavigationEstimate, navCallback)

    a = np.asarray([ ["X","Y","track_fid","track_seg","ele","time"]])
    with open('hei.csv','ab') as fd:
        np.savetxt(fd, a, delimiter=",", fmt="%s")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
