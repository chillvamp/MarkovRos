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
from tf.transformations import  quaternion_from_euler
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker , MarkerArray
import numpy as np
import pandas as pd

global xcl,ycl

def list_2_markers_array(path, ccxyth, deleteall=False):
        
    xythpath=[]
    marker=Marker() 
    markerarray=MarkerArray()    
    for n in path:
        quaternion = quaternion_from_euler(0, 0, ccxyth[n][2])
       
        marker.header.frame_id="/map"
        marker.header.stamp = rospy.Time.now()
        marker.id=n
        marker.type = marker.ARROW
        marker.action = marker.ADD
        if deleteall:
            marker.action= marker.DELETE
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]        
        marker.pose.orientation.w = quaternion[3]
        marker.pose.position.x = ccxyth[n][0]
        marker.pose.position.y = ccxyth[n][1]  
        marker.pose.position.z = 0
        marker.lifetime.nsecs=1
        markerarray.markers.append(marker)
        marker=Marker() 
        
        
        
    return markerarray







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
    def __init__(self,x,y,th=0):
        self.x=x
        self.y=y
        self.th=th
        
        
class grafo (object):
             def __init__(self,nodos,conec):
                 self.nodos=nodos
                 self.conec=conec        

def dijkstra(nodoinicial,nodofinal,graphe):
    

    numnodos= len(graphe.nodos)
    con = graphe.conec
    D= np.ones(numnodos)*np.inf
    Prv= np.ones(numnodos)*np.inf
    V= np.zeros(numnodos)
    a = nodoinicial
    D[a]=0
    Prv[a]=0
    Prv[np.where(con[a]!=np.inf)]=a
    V[a]=1
    Dacc=D[a]
    ########
    D=np.minimum(D,con[a]+D[a])
    cont=0
    sucess=False
    while(sucess==False):
        a = np.argmin(D+np.where (V==1,np.inf, V))
        Dacc=D[a]
        Prv[np.where(D>(con[a]+Dacc) )]=a
        V[a]=1
        D=np.minimum(D,con[a]+Dacc)
        if (a== nodofinal):
            print("RUTA CALCULADA ")
            sucess=True
    rutainv=[]
    rutainv.append(nodofinal)
    while(rutainv[-1]!=nodoinicial):
        prv=Prv[int(rutainv[-1])]
        rutainv.append(prv)

    ruta=[]
    for n in reversed(rutainv):
        ruta.append(n)
    return(ruta)
    
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
     
     global path,thetacl,nxt_pnt,xcl,ycl
     
     if (len(path)==0):
     
         thetacl=0
         nxt_pnt= punto
              
         xcl =punto.point.x
         ycl =punto.point.y
        
         xyth_cl=np.array((xcl,ycl,0))##### ORIENTATION IGNORED
         _,xythcuant= quantized(xyth,ccxyth)
         _,xythclcuant= quantized(xyth_cl,ccxyth)
         print('nodo inicial', xythcuant,xythclcuant)
         path=dijkstra(xythcuant,xythclcuant,graphe)
         #path=[66,44,89,0]
         
         print('Dijkstra ccxyth path',path)
         print('Nxt X, NXt Y',nxt_pnt.point.x,nxt_pnt.point.y)
         nxt_pnt.point.x=ccxyth[np.int(path[0]),0]
         nxt_pnt.point.y=ccxyth[np.int(path[0]),1]
         print('Nxt X, NXt Y',nxt_pnt.point.x,nxt_pnt.point.y)
     else:
        print("YA HAY PATH")
        thetacl=0
        nxt_pnt= punto
        xcl =punto.point.x
        ycl =punto.point.y
        xyth_cl=np.array((xcl,ycl,0))##### ORIENTATION IGNORED
        _,xythclcuant= quantized(xyth_cl,ccxyth)
        print('nodo inicial', path[-1],xythclcuant)
        auxpath=dijkstra(path[-1],xythclcuant,graphe)
        path2=path[:-1]+auxpath
        path=path2
        
         

     
"""def readPoint(punto):
     global xcl
     global ycl

     xcl =punto.point.x
     ycl =punto.point.y
"""           
def readSensor(data):
    
     global graphe , markerstopub
     global x,y,th,A,ccxyth,xyth,xythcuant,path
     markerstopub=MarkerArray()
     x_nxt,y_nxt=x,y
     euclD=1
     
     
     #markerstopub= MarkerArray()
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
        
       
        markerstopub= list_2_markers_array(path,ccxyth) 
       
        pub3.publish(markerstopub)
        x_nxt,y_nxt,th_nxt= ccxyth[path[0]]
        xyth_nxt=np.array((x_nxt,y_nxt,th_nxt))
        _,xyth_nxt_cuant= quantized(xyth_nxt,ccxyth)
        euclD=np.linalg.norm(xyth[:2]-xyth_nxt[:2])
        print ("path",path)
        print('im in ' ,xyth ,xythcuant)
        print('nx to',x_nxt,y_nxt, xyth_nxt_cuant)
        print( 'EuclD to dest.',euclD )
       
    #     
        """if (xythcuant in path[1:]):
            killix= path.index(xythcuant)
            print ('SHortuct DETECTED',killix)
            del path[:path.index(xythcuant)]
        """   
        if (xythcuant==xyth_nxt_cuant and  euclD <=.5):
            print('path node chekced ')
            markerstopub= list_2_markers_array(path,ccxyth,True) 
            pub3.publish(markerstopub)
            path.pop(0)
            nxt_pnt.point.x=ccxyth[np.int(path[0]),0]
            nxt_pnt.point.y=ccxyth[np.int(path[0]),1]
            print('Nxt X, NXt Y',nxt_pnt.point.x,nxt_pnt.point.y)
     else:
         print("Need Click")
         x_nxt,y_nxt=0,0
    
     
   
     
     
         
         
         
         
     Fatrx = ( -x + x_nxt)/euclD
     Fatry = ( -y + y_nxt)/euclD
     Fatrth=np.arctan2(Fatry, Fatrx) 
     Fatrth=Fatrth-th
     Fmagat= np.linalg.norm((Fatrx,Fatry))
     #print ('Fatx, Fatry, Fatrth',Fatrx,Fatry,(Fatrth)*180/np.pi )
     Ftotx= Fmag*np.cos(Fth)*.0051   +  3*  Fmagat*np.cos(Fatrth)
     Ftoty= Fmag*np.sin(Fth)*.0051    + 3*   Fmagat*np.sin(Fatrth)
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
     if( abs(Ftotth) < .13) :#or (np.linalg.norm((Fx,Fy)) < 100):
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
            speed.angular.z=-2
            if Ftotth > 1.57:
                print('ang--')
                speed.linear.x=0
                speed.angular.z=-0.5
        if Ftotth > 0:
            if (abs( Ftotth ) < np.pi/2):
                vel=3
                print('open curve')
         
            
            print('Vang+')
            speed.linear.x=vel
            speed.angular.z=2
            if Ftotth < -1.57:
            
                print('ang++')
                speed.linear.x=0
                speed.angular.z=0.5
            
            
        
     
        
     
     
     if (graphe.conec[xythcuant,path[0]] == np.inf):
         print('Route lost Recalculate Dijsktra#####################################3', )
         print('im in ' ,xyth , 'cc', xythcuant)
         print('going to',x_nxt,y_nxt,'xythcuant###########################3', xyth_nxt_cuant)
       
     





speed=Twist()
nxt_pnt=PointStamped()
speed.angular.z=0
def inoutinout():
    global pub3
  
    sub= rospy.Subscriber("/hsrb/odom",Odometry,newOdom)
    sub2=rospy.Subscriber("/hsrb/base_scan",LaserScan,readSensor)
    sub3=rospy.Subscriber("/clicked_point",PointStamped,readPoint)
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    pub2 = rospy.Publisher('/aa/Markov_NXT/', PointStamped, queue_size=1)  
    pub3= rospy.Publisher('aa/Markov_route',MarkerArray)
    rospy.init_node('talker_cmdvel', anonymous=True)
    rate = rospy.Rate(15) # 10hz
    
    while not rospy.is_shutdown():
        hello_str = "Time %s" % rospy.get_time()
        #print("odom",x,y,th)
  #    
        pub.publish(speed)
        pub2.publish(nxt_pnt)
        #print ("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$",markerstopub)
        
        
      
        rate.sleep()

if __name__ == '__main__':
    try:
        global path
        path=[]
        
        inoutinout()
    except rospy.ROSInterruptException:
        pass
