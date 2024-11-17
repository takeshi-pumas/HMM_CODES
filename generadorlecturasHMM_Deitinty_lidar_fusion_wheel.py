# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020
@author: oscar
Modified to use DeiT-Tiny for feature extraction.2024
"""
#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import numpy as np
import cv2
from cv_bridge import CvBridge
import torch
from timm import create_model
from torchvision.transforms import transforms

bridge = CvBridge()
img_width, img_height = 224, 224  # Adjust for DeiT input
kill_node = False
cap_cnt = 0
obs = []

# Load DeiT-Tiny model
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = create_model('deit_tiny_patch16_224', pretrained=True)
model.head = torch.nn.Identity()  # Remove the classification head
model.eval().to(device)

# Preprocessing pipeline
preprocess = transforms.Compose([
    transforms.ToPILImage(),
    transforms.Resize((img_width, img_height)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])  # Match DeiT normalization
])

def roll_from_quaternion(q):
    sinr_cosp = 2 * (q[1] * q[0] + q[3] * q[2])
    cosr_cosp = 1 - 2 * (q[2] * q[2] + q[1] * q[1])
    return np.arctan2(sinr_cosp, cosr_cosp)

def callback(img_msg, odometria, laser):
    global cap_cnt, model

    # Process LiDAR readings
    lec = np.asarray(laser.ranges)
    lec[np.isinf(lec)] = 13.5
    lec_str = ','.join(map(str, lec))
    print(f'lidar reading length {len(lec)}')
    # Process odometry
    xyth = np.asarray((odometria.pose.pose.position.x,
                       odometria.pose.pose.position.y,
                       euler_from_quaternion([
                           odometria.pose.pose.orientation.x,
                           odometria.pose.pose.orientation.y,
                           odometria.pose.pose.orientation.z,
                           odometria.pose.pose.orientation.w])[2]))
    rospy.loginfo(f"Odometry: {xyth}")

    # Process image
    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    img_tensor = preprocess(cv2_img).unsqueeze(0).to(device)

    # Feature extraction
    with torch.no_grad():
        features = model(img_tensor).cpu().numpy().flatten()
    print(f"Features shape: {features.shape}, Number of features: {features.size}")

    # Combine features and save
    feature_str = ','.join(map(str, features))
    texto = f"{feature_str},{lec_str},{xyth[0]},{xyth[1]},{xyth[2]}"

    with open('validation_deit_lidar_odom.txt', 'a') as out:
        out.write(texto + '\n')

def listener():
    rospy.init_node('listener', anonymous=True)
    image = message_filters.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image)
    odom = message_filters.Subscriber('/hsrb/wheel_odom', Odometry)
    laser = message_filters.Subscriber('/hsrb/base_scan', LaserScan)
    ats = message_filters.ApproximateTimeSynchronizer([image, odom, laser], queue_size=5, slop=0.1, allow_headerless=True)
    ats.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
