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
sys.path.append('fanuc_ethernet_ip_drivers/src')
from robot_controller import robot
import paho.mqtt.client as mqtt_client
import json


dj_ip = '10.8.4.16'
bill_ip = '10.8.4.6'
z_table=125

dj=robot(dj_ip)
# open_gripper_params = {
#     "width_in_mm":138,
#     "force_in_newtons":40,
#     "wait":True
# }
# close_onto_dice_params = {
#         "width_in_mm":76,
#         "force_in_newtons":40,
#         "wait":True
# }
home_position = [0.0,0.0,0.0,0.0,-90.0,30]
dj.write_joint_pose(home_position)
def home_robot():
    dj.write_joint_pose(home_position)
    time.sleep(1)
def pickup_die(x, y, z=z_table):
    dj.write_cartesian_position([x, y, z + 100.0, -179.0, 0, -130.0])
    time.sleep(0.25)
    dj.write_cartesian_position([x, y, z, -179.0, 0, -130.0])
    time.sleep(0.25)
    dj.schunk_gripper('close')
    time.sleep(1)
    time.sleep(0.25)
    dj.write_cartesian_position([x, y, z + 100.0, -179.0, 0, -130.0])
    time.sleep(0.25)

def save_H_mtx(H):
    np.savetxt("homography_mtx_dj.txt", H)

def read_H_mtx():
    H_loaded = np.loadtxt("homography_mtx_dj.txt")
    return H_loaded

def find_die_in_image():
    # Read in H mtx
    H = read_H_mtx()
    # Take photo of die
    img=detect_and_count.take_photo()
    x,y,_,_ = detect_and_count.find_die(img)
    # Use it to convert a pixel (x,y) to real (X,Y)
    pixel = np.array([x, y, 1], dtype=np.float32)
    real = H @ pixel
    real /= real[2] # normalize
    X, Y = real[0], real[1]
    print(f"Pixel coordinates: ({x}, {y})")
    print(f"Real-world coordinates: ({X}, {Y})")
    return X, Y
    # Send robot to X,Y and pick up

def find_homography_matrix():
    pickup_die(838.9059448242188, -353.26605224609375, 123.34469604492188)
    #dj.write_cartesian_position([838.9059448242188, -353.26605224609375, 123.34469604492188, -179.0, 0, -130.0])
    offset = 300.0
    px = 550.0
    py = -550.0
    dj.write_cartesian_position([px, py, z_table, -179.0, 0, -130.0])
    time.sleep(1)
    dj.write_cartesian_position([px, py, z_table-100, -179.0, 0, -130.0])
    dj.schunk_gripper('open')
    time.sleep(1)
    dj.write_cartesian_position([px, py, z_table, -179.0, 0, -130.0])
    home_robot()
    real_points = np.array([
    [px,py],
    [px,py - offset],
    [px - offset,py - offset],
    [px - offset,py],
    ], dtype=np.float32)
    
    # Corresponding real-world points (robot coordinates)
    image_points = np.array([
    [0,0],
    [0,0],
    [0,0],
    [0,0]
    ], dtype=np.float32)

    for i in range(0,4):
        # Move the robot out of the way
        img=detect_and_count.take_photo()
        x,y,w,h = detect_and_count.find_die(img)
        # Save die center coordinates
        image_points[i]=[(x+(0.5*w)),(y+(0.5*h))]
        # Pick die back up
        pickup_die(float(real_points[(i) % 4][0]), float(real_points[(i) % 4][1]))
        dj.write_cartesian_position([float(real_points[(i) % 4][0]), float(real_points[(i) % 4][1]), 223.70266723632812, -179.0, 0, -130.0])
        dj.write_cartesian_position([float(real_points[(i + 1) % 4][0]), float(real_points[(i + 1) % 4][1]), 123.70266723632812, -179.0, 0, -130.0])
        time.sleep(1)
        dj.write_cartesian_position([float(real_points[(i+1) % 4][0]), float(real_points[(i+1) % 4][1]), 123.70266723632812, -179.0, 0, -130.0])
        dj.schunk_gripper('open')
        time.sleep(1)
        dj.write_cartesian_position([float(real_points[(i + 1) % 4][0]), float(real_points[(i + 1) % 4][1]), 223.70266723632812, -179.0, 0, -130.0])
        home_robot()
    
    print("successfully gathered points")
    # Compute homography matrix
    H, _ = cv2.findHomography(image_points, real_points)
    save_H_mtx(H)
    print("Homography Matrix:")
    print(H)

def main():
    dj.set_speed(300)
    dj.schunk_gripper('open')
    time.sleep(1)
    home_robot()
    #find_homography_matrix()
    find_homography_matrix()

   #  x_offset = 47.0
   #  y_offset = 52.0
   #  x, y = find_die_in_image()

   #  pickup_die(x + x_offset, y + y_offset)
main()
