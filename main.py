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
   
   # cluster_free = 0
   # while not cluster_free:
   #    img=detect_and_count.take_photo()
   #    cluster_free=homography_mtx.locate_die(img)
   #    time.sleep(2)
   # img_coords = detect_and_count.read_dice_pixel_coords()
   # print("---Image Coordinates---")
   # print(img_coords)
   # # for coord in img_coords:
   # for i in range(0,len(img_coords)):
   #    # x,y,w,h
   #    (x,y)=homography_mtx.convert_pix_to_robot_coords(img_coords[i][0],img_coords[i][1],img_coords[i][2],img_coords[i][3])
   #    coords.append((float(x),float(y)))
   # print("---Real World Coordinates---")
   # print(coords)
   # cv2.waitKey(0)
   # cv2.destroyAllWindows()
   
main()
def pickup_die(x, y, z=122.70266723632812):
    dj.write_cartesian_position([x, y, z + 100.0, -179.5, 0, 120])
    time.sleep(0.25)
    dj.write_cartesian_position([x, y, z, -179.5, 0, 120])
    time.sleep(0.25)
    dj.schunk_gripper('close')
    time.sleep(0.25)
    dj.write_cartesian_position([x, y, z + 100.0, -179.5, 0, 120])
    time.sleep(0.25)

def find_homography_matrix_bill():
    pickup_die(844.6377563476562, 353.57177734375, 125.70266723632812)
    DJ.write_cartesian_position([844.6377563476562, 353.57177734375, 125.70266723632812, -179.5, 0, 120])
    offset = 300.0
    px = 450.0
    py = 650.0
    DJ.write_cartesian_position([px, py, 225.70266723632812, -179.5, 0, 120])
    time.sleep(1)
    DJ.write_cartesian_position([px, py, 125.70266723632812, -179.5, 0, 120])
    DJ.schunk_gripper('open')
    DJ.write_cartesian_position([px, py, 222.70266723632812, -179.5, 0, 120])
    home_robot()
    real_points = np.array([
    [px,py],
    [px,py + offset],
    [px - offset,py + offset],
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
        DJ.write_cartesian_position([float(real_points[(i) % 4][0]), float(real_points[(i) % 4][1]), 222.70266723632812, -179.5, 0, 120])
        DJ.write_cartesian_position([float(real_points[(i + 1) % 4][0]), float(real_points[(i + 1) % 4][1]), 125.70266723632812, -179.5, 0, 120])
        time.sleep(1)
        DJ.write_cartesian_position([float(real_points[(i+1) % 4][0]), float(real_points[(i+1) % 4][1]), 125.70266723632812, -179.5, 0, 120])
        DJ.schunk_gripper('open')
        DJ.write_cartesian_position([float(real_points[(i + 1) % 4][0]), float(real_points[(i + 1) % 4][1]), 222.70266723632812, -179.5, 0, 120])
        home_robot()
    

    # Compute homography matrix
    H, _ = cv2.findHomography(image_points, real_points)
    homography_mtx.save_H_mtx(H)
    print("Homography Matrix:")
    print(H)
