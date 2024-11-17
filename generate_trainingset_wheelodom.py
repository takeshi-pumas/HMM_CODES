#!/usr/bin/env python3
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
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion




def callback(laser,odometria):
    
        #######################################################
        #centroides = np.load('centroidesVk.npy')
        
        lec=np.asarray(laser.ranges)
        
        lec[np.isinf(lec)]=13.5
        lec_str=str(lec[0])+','
        for i in range (1,len(lec)):
            lec_str=lec_str+str(lec[i])+',' 
            #print (lec_str)
        lec.reshape(len(laser.ranges),1 )
        
        ############################################################    

        
        xy  = np.asarray((odometria.pose.pose.position.x,odometria.pose.pose.position.y))
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
        




        with  open('lecs_odom.txt' , 'a') as out:
            out.write (lec_str +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +'\n' )
            #out.write (lec_str +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +","+  str(symbol)+','+str(symbolxy)+'\n' )
        
	
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    #odom  = message_filters.Subscriber('/hsrb/odom',Odometry)
    odom  = message_filters.Subscriber('/hsrb/wheel_odom',Odometry)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,odom],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
