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
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
global cont
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


def callback(frame,pose):
    
    
        #######################################################
        centroides = np.load('ccvk.npy')
        ccxyth= np.load('ccxyth.npy')
        cv2_img = bridge.imgmsg_to_cv2(frame, "bgr8")
        gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
    
        radius = 4
        num_corners=100
        corners=cv2.goodFeaturesToTrack(gray,num_corners,.01,10)
        str_corners= ''
        for corner in corners:
            str_corners=str_corners+ str(corner[0][0])+ ','+str(corner[0][1])
        
        
        
        
        """
        lec=np.asarray(laser.ranges)
       
        lec[np.isinf(lec)]=13.5
        
        lec.reshape(len(laser.ranges),1 )
        cont.append(lec)
        """
#        
        quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        #symbol= np.power(lec.T-centroides,2).sum(axis=1,keepdims=True).argmax()
        xyth= np.asarray((pose.pose.position.x,pose.pose.position.y,euler[2]))
       
        xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
        print(corners.shape,'est',xythcuant)
        #print('lec vk'+str(symbol)+' Pose ('  +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +') , ccxyth[ '+str(xythcuant)+']='+str(ccxyth[xythcuant]) + '\n')
        with  open('lecturas_img_conodometria.txt' , 'a') as out:
            out.write (str_corners+","+str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])    +'\n' )
            










        
      
        
        ############################################################
        #ccxyth=np.load('HMM/ccxyth.npy')#xy = np.load('ccxy.npy')
        #cc= ccxyth[:,1:]
        
        
        """xy  = np.asarray((odometria.pose.pose.position.x,odometria.pose.pose.position.y))
        quaternion = (
        odometria.pose.pose.orientation.x,
        odometria.pose.pose.orientation.y,
        odometria.pose.pose.orientation.z,
        odometria.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]
        
        xyth= np.asarray((odometria.pose.pose.position.x,odometria.pose.pose.position.y,euler[2]))
        print(str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2]) +' \n')
        #symbolxy =np.power(xy-ccxy,2).sum(axis=1 ,keepdims=True).argmax()   
        #symbolxy =np.power(xyth-ccxyth,2).sum(axis=1 ,keepdims=True).argmax()   
        #lecodom=np.asarray((xyth.pose.pose.position.x , xyth.pose.pose.position.y, xyth.twist.twist.angular.z).reshape(1,3)
        # lecodom= np.power(xyth[0:2]-centxyth[:,0:2],2).sum(axis=1,keepdims=True).argmax()
        #velocidad= np.asarray((twist.linear.x,twist.angular.z))
        #print(lec_str+str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +","+  str(symbol)+','+str(symbolxy)+','+str(velocidad[0])+','+str(velocidad[1])+' \n' )
        with  open('lecturasconodometria.txt' , 'a') as out:
            out.write (lec_str +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +'\n' )
            #out.write (lec_str +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +","+  str(symbol)+','+str(symbolxy)+'\n' )
        """
	
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
   
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/hand_camera/image_raw',Image)
    
    #pose  = message_filters.Subscriber('/navigation/localization/amcl_pose',PoseWithCovarianceStamped)#TAKESHI REAL
    pose  = message_filters.Subscriber('/hsrb/base_pose',PoseStamped)#TAKESHI GAZEBO
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,pose],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    cont=[]    
    print("cont",cont)
    listener()
