#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019

@author: oscar
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist , PointStamped , Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd




def Markov_A_2_grafo(A,ccxyth):
    dists=np.zeros(A.shape)
    for i in range(A.shape[0]):
        for j in range (A.shape[1]):
            if A[i,j]!=0 :
                dists[i,j]= np.linalg.norm(ccxyth[i]-ccxyth[j])    
    
    
    con = np.where(dists==0,np.inf,dists)
    graphe2=grafo(ccxyth,con)
    return graphe2


class node(object):
    def __init__(self,x,y):
        self.x=x
        self.y=y
        
        
class grafo (object):
             def __init__(self,nodos,conec):
                 self.nodos=nodos
                 self.conec=conec        

def dijkstra(nodoinicial,nodofinal,graphe):

##INIT    
    numnodos= len(graphe.nodos)
    con = graphe.conec
    D= np.ones(numnodos)*np.inf
    V= np.zeros(numnodos)
    ruta=[]
    D[nodoinicial]=0
    a = nodoinicial
    ruta.append(a)
 ###FIN INIT   
    jaja=pd.Series(con[a])
    cona=jaja[jaja != np.inf]
    V[a]=1
    for c in cona.index:
        if cona [c] < D[c]:
            D[c] = cona[c]
            Daux = pd.Series(D)
    a= Daux[V!=1].idxmin()
    ruta.append(a)
#####FF
    while a != nodofinal:
        jaja=pd.Series(con[a])
        cona=jaja[jaja != np.inf]
        V[a]=1
        for c in cona.index:
            if cona[c]+D[a]  < D[c] :
                D[c] = cona[c] + D[a]
        Daux= pd.Series(D)
        a= Daux[V!=1].idxmin()
        ruta.append(a)
    return (ruta)
def quantized(xyth,ccxyth):
    xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
    x,y=ccxyth[xythcuant,:2]
    return ((x,y),(xythcuant))
def normalize(x,y):
    xn= x/np.linalg.norm((x,y))
    yn= y/np.linalg.norm((x,y))
    return ((xn,yn))

def newOdom (msg):
    global x
    global y
    global th
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    th=euler[2]
    
def readPoint(punto):
     
     global xcl,ycl,path,thetacl
     thetacl=0
    
     xcl =punto.point.x
     ycl =punto.point.y
     xyth_cl=np.array((xcl,ycl,0))##### ORIENTATION IGNORED
     
     _,xythclcuant= quantized(xyth_cl,ccxyth)
     print('nodo inicial', xythcuant,xythclcuant)
     path=dijkstra(xythcuant,xythclcuant,graphe)
     print('Dijkstra ccxyth path',path)
     
  
     
"""def readPoint(punto):
     global xcl
     global ycl

     xcl =punto.point.x
     ycl =punto.point.y
"""           
def readSensor(data):
    
     global graphe
     global x,y,th,A,ccxyth,xyth,xythcuant
     
    
    
     lec=np.asarray(data.ranges)
     lec[np.isinf(lec)]=13.5
     ccxyth=np.load('ccxyth.npy')
     A=np.load('A.npy')
     graphe= Markov_A_2_grafo(A,ccxyth)        
     xyth= np.asarray((x,y,th)) 
     _,xythcuant= quantized(xyth,ccxyth)
     
     """if (graphe.conec[xythcuant,path[0]] == np.inf):
         print("Recalculating Dijsktra")
         path=dijkstra(xythcuant,xythclcuant,graphe)
     """
     
     
    
     print ("path",path)
     Fx, Fy,Fth = 0.001,0.001,0
     deltaang=4.7124/len(data.ranges)
     laserdegs=  np.arange(-2.3562,2.3562,deltaang)
     Fx=0
     Fy = 0.001
     for i,deg in enumerate(laserdegs):
         if (lec[i] <2.61):
             Fx = Fx + (1/lec[i])**2 * np.cos(deg)
             Fy = Fy + (1/lec[i])**2 * np.sin(deg)
     Fth= np.arctan2(Fy,(Fx+.000000000001))+np.pi
     Fmag= np.linalg.norm((Fx,Fy))
     
     
        
     if (len (path)>0):
        x_nxt,y_nxt,th_nxt= ccxyth[path[0]]
        xyth_nxt=np.array((x_nxt,y_nxt,th_nxt))
        _,xyth_nxt_cuant= quantized(xyth_nxt,ccxyth)
        euclD=np.linalg.norm(xyth[:2]-xyth_nxt[:2])
        
        print('im in ' ,xyth , 'cc', xythcuant)
        print('going to',x_nxt,y_nxt,'xythcuant', xyth_nxt_cuant)
        print( 'EuclD to dest.',euclD )
    #     
        if (xythcuant in path[1:]):
            killix= path.index(xythcuant)
            print ('SHortuct DETECTED',killix)
            del path[:path.index(xythcuant)]
           
        if (xythcuant==xyth_nxt_cuant or  euclD <=.35):
            print('path node chekced ')
            path.pop(0)
         
    
     
   
     
     
         
     if (len (path)==0):
         x_nxt,ynxt = xcl,ycl
         
         
         
     Fatrx =  1/( -x + x_nxt)
     Fatry = 1/( -y + y_nxt)
     Fatrth=np.arctan2(Fatry, Fatrx) 
     Fatrth=Fatrth-th
     Fmagat= np.linalg.norm((Fatrx,Fatry))
     #print ('Fatx, Fatry, Fatrth',Fatrx,Fatry,(Fatrth)*180/np.pi )
     Ftotx= Fmag*np.cos(Fth)*.0051   +    Fmagat*np.cos(Fatrth)
     Ftoty= Fmag*np.sin(Fth)*.0051    +    Fmagat*np.sin(Fatrth)
     Ftotth=np.arctan2(Ftoty,Ftotx)
     
     if ( Ftotth> np.pi ):
         Ftotth=       -np.pi-    (Ftotth-np.pi)
    
     if (Ftotth < -np.pi):
         Ftotth= (Ftotth     +2 *np.pi)
     
             
     #print('Ftotxy',Ftotx,Ftoty,Ftotth*180/np.pi)
     """Fatmag=np.linalg.norm((Fatrx,Fatry))
     Fmag=np.linalg.norm((Fx,Fy))
     print ("theta robot",th*180/3.1416,'---------------------------')
     print ('fasorFatrth',np.linalg.norm((Fatrx,Fatry)),(Fatrth)*180/3.1416 )
     print ("FXATR,FYATR",Fatrx,Fatry)
     print ('fasorFrepth',np.linalg.norm((Fx,Fy)),Fth*180/3.1416)
     print ("Frepx,Frepy",Fx,Fy)"""
     
        
     """Fx,Fy= Fmag*np.cos(Fth) , Fmag*np.sin(Fth)   
     Fatrx,Fatry= Fatmag*np.cos(Fatrth) , Fatmag*np.sin(Fatrth) """
    
    
     vel=0 
     if( abs(Ftotth) < .53) :#or (np.linalg.norm((Fx,Fy)) < 100):
         speed.linear.x=5
         print('lin')
         speed.angular.z=0
     
     else:
         
        if Ftotth < 0:
            if (abs( Ftotth ) < np.pi/2):
                vel=3
                print('open curve')
         
            print('Vang-')
            speed.linear.x=vel
            speed.angular.z=-1.5
        if Ftotth > 0:
            if (abs( Ftotth ) < np.pi/2):
                vel=3
                print('open curve')
         
            
            print('Vang+')
            speed.linear.x=0
            speed.angular.z=1.5
        """if Ftotth < -1.57:
            
            print('ang++')
            speed.linear.x=0
            speed.angular.z=0.5
        
            
        
 
        if Ftotth > 1.57:
            print('ang-')
            speed.linear.x=0
            speed.angular.z=-0.5"""
            
        
     
     
     if (graphe.conec[xythcuant,path[0]] == np.inf):
         print('Route lost Recalculate Dijsktra')
       
        
            
       
speed=Twist()
speed.angular.z=0
def inoutinout():
    
    sub= rospy.Subscriber("/hsrb/odom",Odometry,newOdom)
    sub2=rospy.Subscriber("/hsrb/base_scan",LaserScan,readSensor)
    sub3=rospy.Subscriber("/clicked_point",PointStamped,readPoint)
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    pub2 = rospy.Publisher('/hsrb/Markov_next_node/', PointStamped, queue_size=1)    
    rospy.init_node('talker_cmdvel', anonymous=True)
    rate = rospy.Rate(15) # 10hz
    
    while not rospy.is_shutdown():
        hello_str = "Time %s" % rospy.get_time()
        #print("odom",x,y,th)
  #    
        pub.publish(speed)
      
        rate.sleep()

if __name__ == '__main__':
    try:
        global path
        path=[]
        inoutinout()
    except rospy.ROSInterruptException:
        pass
