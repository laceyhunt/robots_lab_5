import detect_and_count
import mqtt
import homography_mtx
from detect_and_count import cv2, np
import time
import sys
driver_dir = '../robotics/FANUC-Ethernet_IP_Drivers/src/'
sys.path.append(driver_dir)
from robot_controller import robot

# drive_path = '10.8.4.6' # Bill (OnRobot)
drive_path = '10.8.4.16' # DJ (Schunk)
# Gripper specs (for OnRobot gripper)
open_width = 100
close_width = 78
force = 40
home_pos_joint = [0,0,0,0,-90,30]

def main():
   print('Setting up...')
   dj = robot(drive_path)
   dj.set_speed(100)
   dj.schunk_gripper('close')
   time.sleep(2)
   dj.schunk_gripper('open')
   time.sleep(2)

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
