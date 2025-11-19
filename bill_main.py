#coding=utf-8
from http import client
import cv2
import numpy as np
import mvsdk
import platform
import detect_and_count
from standardbots import models, StandardBotsRobot
import time
import random
import sys
sys.path.append('../../asn3/fanuc_ethernet_ip_drivers/src')
from robot_controller import robot
import paho.mqtt.client as mqtt_client
import json
import homography_mtx
import mqtt as mqtt_module
die_count_on_table = [0,0,0,0,0,0,0]
# Michael is the broker
broker = '10.8.4.17'
# broker port
port = 1883
# Just a declaration of the global variable
topic = "robot/test"
# Ip of DJ robot
dj_ip = '10.8.4.16'
bill_ip = '10.8.4.6'

# Topic that DJ will publish to
dj_topic = "robot/dj"
theo_topic = "robot/theodore"

bill=robot(bill_ip)
open_gripper_params = {
    "width_in_mm":138,
    "force_in_newtons":40,
    "wait":True
}
close_onto_dice_params = {
        "width_in_mm":76,
        "force_in_newtons":40,
        "wait":True
}

home_position = [0.0,0.0,0.0,0.0,-90.0,-45.0]
bill.write_joint_pose(home_position)

def home_robot():
    bill.write_joint_pose(home_position)
    time.sleep(1)
    
def pickup_die(x, y, z=122.70266723632812, r= 0.0):
    r = 90 - r
    cartesian_pos = bill.read_current_cartesian_pose()
    bill.write_cartesian_position([x, y, z + 100.0, 179.0, cartesian_pos[4], cartesian_pos[5] + r])
    time.sleep(0.25)
    bill.write_cartesian_position([x, y, z, 179.0, cartesian_pos[4], cartesian_pos[5] + r])
    time.sleep(0.25)
    bill.onRobot_gripper(**close_onto_dice_params)
    time.sleep(1)
    time.sleep(0.25)
    bill.write_cartesian_position([x, y, z + 100.0, -179.0, 0, 50.0])
    time.sleep(0.25)

def find_die_in_image():
    # Read in H mtx
    H = homography_mtx.read_H_mtx("homography_mtx_bill.txt")
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
def place_die_on_table(pip=0):
    bill.write_cartesian_position([652.8466186523438 - 100*(pip), -240.7295837402344, 50.60589599609375 + 81*(die_count_on_table[pip]), -179.0, 0, 130.0])
    bill.onRobot_gripper(**open_gripper_params)
    time.sleep(1)
    home_robot()
    die_count_on_table[pip] += 1
def find_homography_matrix():
    pickup_die(838.9059448242188, -353.26605224609375, 123.34469604492188)
    #bill.write_cartesian_position([838.9059448242188, -353.26605224609375, 123.34469604492188, -179.0, 0, 50.0])
    offset = 300.0
    px = 5-130.0
    py = -5-130.0
    bill.write_cartesian_position([px, py, 223.70266723632812, -179.0, 0, 50.0])
    time.sleep(1)
    bill.write_cartesian_position([px, py, 123.70266723632812, -179.0, 0, 50.0])
    bill.onRobot_gripper(**open_gripper_params)
    time.sleep(1)
    bill.write_cartesian_position([px, py, 223.70266723632812, -179.0, 0, 50.0])
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
        bill.write_cartesian_position([float(real_points[(i) % 4][0]), float(real_points[(i) % 4][1]), 223.70266723632812, -179.0, 0, 50.0])
        bill.write_cartesian_position([float(real_points[(i + 1) % 4][0]), float(real_points[(i + 1) % 4][1]), 123.70266723632812, -179.0, 0, 50.0])
        time.sleep(1)
        bill.write_cartesian_position([float(real_points[(i+1) % 4][0]), float(real_points[(i+1) % 4][1]), 123.70266723632812, -179.0, 0, 50.0])
        bill.onRobot_gripper(**open_gripper_params)
        time.sleep(1)
        bill.write_cartesian_position([float(real_points[(i + 1) % 4][0]), float(real_points[(i + 1) % 4][1]), 223.70266723632812, -179.0, 0, 50.0])
        home_robot()
    
    print("successfully gathered points")
    # Compute homography matrix
    H, _ = cv2.findHomography(image_points, real_points)
    homography_mtx.save_H_mtx(H, "homography_mtx_bill.txt")
    print("Homography Matrix:")
    print(H)
def break_cluster(x,y,w,h):
   """break up a cluster

   Args:
       coordinate (Tuple): x,y robot coord of center of cluster
       width (float, optional): width of cluster in pixels. Defaults to None.
       height (_type_, optional): height of cluster in pixels. Defaults to None.
   """
   print("cluster found!")
   if(w>h):
      # go to bottom of table with y=coordinate y
      # Move robot up to x=coordinate x + some 
      bill.onRobot_gripper(**close_onto_dice_params)
      time.sleep(1)
      bill.write_cartesian_position([915.4827880859375, -456.19580078125, 160, 179.0, 0.0, 130.0])
      bill.write_cartesian_position([x - 50, y - 50, 140, 179.0, 0.0, 130.0])
      time.sleep(1)
      bill.onRobot_gripper(**open_gripper_params)
      time.sleep(1)
      bill.write_cartesian_position([x - 50, y - 50, 180, 179.0, 0.0, 130.0])
      home_robot()
   elif(h>=w):
      #570.2075805664062, -213.02224731445312, 166.31466674804688, -179.38247680664062, 1.405871033668518, -134.75782775878906
      # go to side of table with x=coordinate x
      # Move robot sideways to y=coordinate y + some 
      bill.onRobot_gripper(**close_onto_dice_params)
      time.sleep(1)
      bill.write_cartesian_position([570.2075805664062, -213.02224731445312, 160, -179.0, 0.0, 130.0])
      bill.write_cartesian_position([x - 50, y-50, 140, -179.0, 0.0, 130.0])
      time.sleep(1)
      bill.onRobot_gripper(**open_gripper_params)
      time.sleep(1)
      bill.write_cartesian_position([x-50,y-50, 180, -179.0, 0.0, 130.0])
      home_robot()

def main():
    bill.set_speed(300)
    bill.onRobot_gripper(**open_gripper_params)
    time.sleep(1)
    home_robot()
    #subscribe to mqtt topic
    mqtt_module.message_recieved = False
    cluster_free = False
    client = mqtt_module.connect_mqtt()
    client.loop_start()
    while not cluster_free:
        mqtt_module.subscribe(client, mqtt_module.cluster_topic)
        while not mqtt_module.message_recieved:
            time.sleep(0.2)
        mqtt_module.message_recieved = False
        print("recieved mqtt message")
        parsed_msg = mqtt_module.message_i_got
        cluster = parsed_msg["cluster_flag"]
        if cluster == 'bill':
            coordinates = parsed_msg.get("coordinate")
            x,y = homography_mtx.convert_pix_to_robot_coords(coordinates[0], coordinates[1], coordinates[2], coordinates[3], "homography_mtx_bill.txt",47,52)
            break_cluster(x, y, coordinates[2], coordinates[3])
            mqtt_module.publish(client, mqtt_module.package_json([0,0,0,0], "dj"), mqtt_module.cluster_topic)
            time.sleep(2)
        elif cluster == 'none':
            print("No cluster detected")
            print(f"Cluster flag: {cluster}")
            cluster_free = True
        time.sleep(0.2)
    client.loop_stop()
    mqtt_module.subscribe(client, mqtt_module.coord_topic)
    client.loop_start()
    mqtt_module.message_recieved = False
    print("Waiting for coordinates...")
    while not mqtt_module.message_recieved:
        time.sleep(0.2)
    parsed_msg = mqtt_module.message_i_got
    coordinates = parsed_msg.get("coords")
    coords = []
    img_coords = coordinates
    for i in range(0,len(img_coords)):
       # x,y,w,h
       if img_coords[i][0] >= 680:
            (x,y)=homography_mtx.convert_pix_to_robot_coords(img_coords[i][0],img_coords[i][1],img_coords[i][2],img_coords[i][3],"homography_mtx_bill.txt",47,52)
            coords.append((float(x),float(y)))
            pickup_die(x, y, z = 123.0, r = img_coords[i][4])
            home_robot()
            place_die_on_table(pip=int(img_coords[i][5]) if int(img_coords[i][5]) < 7 else 0)
            print("---Grabbable Coordinates---")
            print(x , y )
            #pickup_die(x + 47.0, y + 52.0, z = 123.0, r =0.0)
    #find_homography_matrix()
main()
def recieve_coordinates_mqtt(client):
    message_recieved = False
    mqtt_module.subscribe(client, 'topics/coords')
    while not message_recieved:
        time.sleep(0.2)
    client.loop_stop()