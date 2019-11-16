#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019

@author: oscar
"""

from geometry_msgs.msg import Quaternion
import numpy as np
import rospy
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from utils import  viterbi
from utils import forw_alg
from utils import backw_alg
global o_k
global buf_vit
o_k=[]
buf_vit=100




class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI  
global Modelo1
A, B, PI= np.load('A.npy') , np.load('B.npy') , np.load('PI.npy')

Modelo1= HMM(A,B,PI)


def callback(laser,pose):
    
        #######################################################
        centroides = np.load('ccvk.npy')
        ccxyth= np.load('ccxyth.npy')
        
        
        
        
        lec=np.asarray(laser.ranges)
        #print (lec.shape)
        lec[np.isinf(lec)]=13.5
        lec_str=str(lec[0])+','
        for i in range (1,len(lec)):
            lec_str=lec_str+str(lec[i])+',' 
            #print (lec_str)
        lec.reshape(len(laser.ranges),1 )
        
       
        
#        
        quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]
        symbol= np.power(lec.T-centroides,2).sum(axis=1,keepdims=True).argmax()
        if len(o_k) >=buf_vit:
            o_k.pop(0)
        o_k.append(symbol)
        xyth= np.asarray((pose.pose.position.x,pose.pose.position.y,euler[2]))
       
        xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
      
        
        if (len(o_k)== buf_vit):
            #print (o_k)
            vit_est= viterbi(o_k,Modelo1,Modelo1.PI)
            alpha= forw_alg(o_k,Modelo1)
            print ('Most likely state seq given O',vit_est)
            print ('P State given O, M',alpha[:,-1])
            
            #vit_est= viterbi(o_k,Modelo2,Modelo2.PI)
            #print ('#############################',vit_est)
        print('lec vk'+str(symbol)+' Pose ('  +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +') , ccxyth[ '+str(xythcuant)+']='+str(ccxyth[xythcuant]) + '\n')
        
	
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
   
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    
    #pose  = message_filters.Subscriber('/navigation/localization/amcl_pose',PoseWithCovarianceStamped)#TAKESHI REAL
    pose  = message_filters.Subscriber('/hsrb/base_pose',PoseStamped)#TAKESHI GAZEBO
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,pose],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    
   
    listener()
