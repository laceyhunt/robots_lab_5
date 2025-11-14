#coding=utf-8
import cv2
import numpy as np
import mvsdk
import platform
import detect_and_count
from standardbots import models, StandardBotsRobot
import time
import random
import sys
sys.path.append('../robotics/FANUC-Ethernet_IP_Drivers/src/')
from robot_controller import robot
import paho.mqtt.client as mqtt_client
import json
import homography_mtx


# Ip of DJ robot
dj_ip = '10.8.4.16'
x_offset = 45.0
y_offset = 50.0
dj=robot(dj_ip)
dj.set_speed(300)
home_pos_joint = [0.0,0.0,0.0,0.0,-90.0,30.0]
z_pos=123


def main():
    dj.schunk_gripper('open')
    dj.write_joint_pose(home_pos_joint)
    
    dj.schunk_gripper('close')
    dj.schunk_gripper('open')

    dj.write_joint_pose(home_pos_joint)

    coords=[]
    
    cluster_free = 0
    while not cluster_free:
       img=detect_and_count.take_photo()
       cluster_free=homography_mtx.locate_die(img)
       time.sleep(2)
    img_coords = detect_and_count.read_dice_pixel_coords()
    print("---Image Coordinates---")
    print(img_coords)
    # for coord in img_coords:
    for i in range(0,len(img_coords)):
       # x,y,w,h
       (x,y)=homography_mtx.convert_pix_to_robot_coords(img_coords[i][0],img_coords[i][1],img_coords[i][2],img_coords[i][3])
       coords.append((float(x),float(y)))
    print("---Real World Coordinates---")
    print(coords)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
        
main()
