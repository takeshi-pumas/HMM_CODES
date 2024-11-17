# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020

@author: oscar
"""

#!/usr/bin/env python3
    
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image , LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


import os
import sys


import cv2
import tensorflow
from tensorflow.keras.applications.inception_resnet_v2 import InceptionResNetV2, preprocess_input
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
kill_node=False
cap_cnt=0
obs=[]
img_width,img_height=600,600
model=InceptionResNetV2(weights='imagenet',include_top=False, input_shape=(img_width, img_height, 3))
model_used='resnet'


def roll_from_quaternion(q):
    sinr_cosp = 2 * (q[1] * q[0] + q[3] * q[2])
    cosr_cosp = 1 - 2 * (q[2] * q[2] + q[1] * q[1])
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    return roll

def callback(img_msg,odometria,laser):
    global  cap_cnt , model

    lec=np.asarray(laser.ranges)
    print (lec.shape)
    lec[np.isinf(lec)]=13.5
    lec_str=str(lec[0])+','
    for i in range (1,len(lec)):
        lec_str=lec_str+str(lec[i])+',' 
    #print (lec_str)

    """print (pose.pose.orientation)
                quaternion = (
                    
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w)
                
                #euler = euler_from_quaternion(quaternion)
                roll = roll_from_quaternion(quaternion)"""
    
        #symbol= np.power(lec.T-centroides,2).sum(axis=1,keepdims=True).argmax()
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


    
    

    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    img_resized=cv2.resize(cv2_img,(img_width,img_height))
    inp_img= np.expand_dims(img_resized ,axis=0)
    o = model.predict(inp_img)[0,0,0,:]
    print (o.shape)
    texto= ''
    for entry in o:
        texto+= str(entry)+','
    texto=texto + lec_str + str(xyth[0])+','+str(xyth[1])+','+str(xyth[2])

    with  open('validation_resnet_lidar_odom.txt' , 'a') as out:

            out.write (texto+'\n' )# +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +'\n' )

        

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    rospy.init_node('listener', anonymous=True)
    image= message_filters.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image)
    #pose  = message_filters.Subscriber('/hsrb/base_pose',PoseStamped)#TAKESHI GAZEBO
    odom  = message_filters.Subscriber('/hsrb/wheel_odom',Odometry)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    ats= message_filters.ApproximateTimeSynchronizer([image, odom , symbol],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    
    #pose  = message_filters.Subscriber('/navigation/localization/amcl_pose',PoseWithCovarianceStamped)#TAKESHI REAL
    #cv2.namedWindow("SHI TOMASI", 1)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



    
    
    #rospy.Subscriber("/hsrb/hand_camera/image_raw", Image, callback)
    #rospy.Subscriber("/hsrb/base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    listener()
