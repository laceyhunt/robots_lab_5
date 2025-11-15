#coding=utf-8
import cv2
import homography_mtx
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

# Ip of DJ robot
dj_ip = '10.8.4.16'
x_offset = 55.0
y_offset = -48
die_size = 88

# x orig is 328
# actual is 342.239

# y orig is 951.7
# actual is 959.740


rot_offset = 52.827

# at 59.589
# needs to be 86.517

dj=robot(dj_ip)
dj.set_speed(300)
home_pos_joint = [0.0,0.0,0.0,0.0,-90.0,30.0]
z_table=125
place_back=[839,-24,66]

# MAX:+90,-150 for wrist

def pick_and_place(loc_1,rot,die_num,loc_2=place_back.copy()):
    loc_2[0]-=(die_size*(die_num-1))
    angle=rot+rot_offset
    # Open grip
    dj.schunk_gripper('open')
    # above loc 1
    pos=[loc_1[0],loc_1[1],z_table+100,-179,0.0,angle]
    dj.write_cartesian_position(pos)
    # on loc 1
    pos=[loc_1[0],loc_1[1],z_table,-179,0.0,angle]
    dj.write_cartesian_position(pos)
    # Close grip
    dj.schunk_gripper('close')
    # above pos 1
    pos=[loc_1[0],loc_1[1],z_table+100,-179,0.0,angle]
    dj.write_cartesian_position(pos)
    
    # above loc 2
    pos=[loc_2[0],loc_2[1],loc_2[2]+100,-179,0.0,28]
    dj.write_cartesian_position(pos)
    # on loc 2
    pos=[loc_2[0],loc_2[1],loc_2[2],-179,0.0,28]
    dj.write_cartesian_position(pos)
    # Open grip
    dj.schunk_gripper('open')
    # above loc 2
    pos=[loc_2[0],loc_2[1],loc_2[2]+100,-179,0.0,28]
    dj.write_cartesian_position(pos)
    
    

def main():
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
    num_die=0
    for i in range(0,len(img_coords)):
        # x,y,w,h,r
        if(img_coords[i][0]<610):
            num_die+=1
            (x,y)=homography_mtx.convert_pix_to_robot_coords(img_coords[i][0],img_coords[i][1],img_coords[i][2],img_coords[i][3],"homography_mtx_DJ.txt",x_offset=x_offset,y_offset=y_offset)
            coords.append((float(x),float(y)))
            loc=[float(x),float(y)]
            # pick_and_place(loc,img_coords[i][4],num_die)
    print("---Real World Coordinates---")
    print(coords)
    # dj.write_joint_pose(home_pos_joint)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

main()



# 1.6 is orig (img)
# 27.535 is actual

