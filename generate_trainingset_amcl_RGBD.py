#!/usr/bin/env python3n
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
from sensor_msgs.msg import Image as ImageMsg, PointCloud2
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
import tf
import ros_numpy
import cv2
from sklearn.cluster import KMeans
class RGBD():
    def __init__(self):
        self._br = tf.TransformBroadcaster()
        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",

            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._xyz = [0, 0, 0]
        self._frame_name = None

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = \
            self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]
        self._region = \
            (self._h_image > self._h_min) & (self._h_image < self._h_max)
        if not np.any(self._region):
            return

        (y_idx, x_idx) = np.where(self._region)
        x = np.average(self._points_data['x'][y_idx, x_idx])
        y = np.average(self._points_data['y'][y_idx, x_idx])
        z = np.average(self._points_data['z'][y_idx, x_idx])
        self._xyz = [y, x, z]
        if self._frame_name is None:
            return

        self._br.sendTransform(
            (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
            self._frame_name,
            msg.header.frame_id)

    def get_image(self):
        return self._image_data

    def get_points(self):
        return self._points_data


def callback(laser,odometria):
    
        #######################################################
        #centroides = np.load('centroidesVk.npy')
        points=rgbd.get_points()
        if points != None:
            pts=[]
            for i in range(points['x'].shape[0]):
                for j in range(points['x'].shape[1]):
                    #CREATE FEATURE VECTOR
                    pt=np.asarray((points['x'][i,j],points['y'][i,j],points['z'][i,j]))
                    if np.isnan(pt).any():pass
                    else:pts.append(pt)
        
            pts_array=np.asarray(pts)    
            kmeans.fit(pts_array)
            str_cc=''
            for i in kmeans.cluster_centers_:
                str_cc+=str(i)

            print ('ccs',str_cc)
            
            



        lec=np.asarray(laser.ranges)
        
        
        lec[np.isinf(lec)]=13.5
        lec_str=str(lec[0])+','
        for i in range (1,len(lec)):
            lec_str=lec_str+str(lec[i])+',' 
            #print (lec_str)
        lec.reshape(len(laser.ranges),1 )


        #print (range (1,len(lec)))
        #symbol= np.power(lec.T-centroides,2).sum(axis=1,keepdims=True).argmax()
        ############################################################
        #ccxyth=np.load('HMM/ccxyth.npy')#xy = np.load('ccxy.npy')
        #cc= ccxyth[:,1:]

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
        print ('rueda', xyth)

        try:
            pose,quat=  listener.lookupTransform('map','base_footprint',rospy.Time())
            x,y = pose[0], pose [1]
            euler = euler_from_quaternion(quat)
            th=euler[2]
            print('AMCL'+str(x)+","+ str(y)  +","+str(euler[2]) +' \n')
            with  open('lecs_odom_ptcld.txt' , 'a') as out:
                #out.write (lec_str +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +'\n' )
                #out.write (lec_str +str(x)+","+ str(y)  +","+str(euler[2])  +'\n' )
                out.write (str_cc +str(x)+","+ str(y)  +","+str(euler[2])  +'\n' )
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print ('notf yet')
            x,y,th=0,0,0

        #symbolxy =np.power(xy-ccxy,2).sum(axis=1 ,keepdims=True).argmin()   
        #symbolxy =np.power(xyth-ccxyth,2).sum(axis=1 ,keepdims=True).argmax()   
        #lecodom=np.asarray((xyth.pose.pose.position.x , xyth.pose.pose.position.y, xyth.twist.twist.angular.z).reshape(1,3)
        # lecodom= np.power(xyth[0:2]-centxyth[:,0:2],2).sum(axis=1,keepdims=True).argmax()
        #velocidad= np.asarray((twist.linear.x,twist.angular.z))
        #print(lec_str+str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +","+  str(symbol)+','+str(symbolxy)+','+str(velocidad[0])+','+str(velocidad[1])+' \n' )
        

        with  open('lecs_odom.txt' , 'a') as out:
            out.write (lec_str +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +'\n' )
            
        
	
def listener():
    global listener , rgbd  ,kmeans
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    listener = tf.TransformListener()
    rgbd=RGBD()
    
    kmeans= KMeans()
    #odom  = message_filters.Subscriber('/hsrb/odom',Odometry)
    odom  = message_filters.Subscriber('/hsrb/wheel_odom',Odometry)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,odom],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
