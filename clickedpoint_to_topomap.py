# -*- coding: utf-8 -*-
"""
Created on Tue Oct  8 12:56:23 2019

@author: oscar
"""

5#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019

@author: oscar
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist , PointStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np

def normalize(x,y):
    xn= x/np.linalg.norm((x,y))
    yn= y/np.linalg.norm((x,y))
    return ((xn,yn))


def readPoint(punto):
     global xcl
     global ycl
     
     clkd_points.append((xcl,ycl))
     xcl =punto.point.x
     ycl =punto.point.y
     print("punto",len(clkd_points),clkd_points)
     
     np.save("manual_ccxy.npy",np.array(clkd_points))
           

def clickedpoint_to_topomap():
    global clkd_points
    clkd_points=[]
   
    sub3=rospy.Subscriber("/clicked_point",PointStamped,readPoint)
    rospy.init_node('clickedpoint_to_topomap', anonymous=True)
    rate = rospy.Rate(15) # 10hz
    while not rospy.is_shutdown():
        hello_str = "Time %s" % rospy.get_time()
        #print("odom",x,y,th)
  #    
        
        rate.sleep()

if __name__ == '__main__':
    try:
        clickedpoint_to_topomap()
    except rospy.ROSInterruptException:
        pass
