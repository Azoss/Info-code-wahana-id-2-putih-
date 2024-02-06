#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
import tf
import cv2
import numpy as np
import apriltag
import math
#from pynput import keyboard
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn, PositionTarget
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavcontroller2 import *

pi_2 = 3.141592654 / 2.0
        


def px_meter (fov, res, alt):
        return ((alt * math.tan(math.radians(fov/2))) / (res/2))

def uji1():
    # Open webcam
    camSet = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
    mav = MavController()
    rospy.sleep(4)
    Kp_x = Kp_y = 0.04
    Ki_x = Ki_y = 0.04
    Kd_x = Kd_y = 0.04
    pid_x = PIDController(Kp_x, Ki_x, Kd_x)
    pid_y = PIDController(Kp_y, Ki_y, Kd_y)
    mav.takeoff(1.0)
    rospy.sleep(10)
    print("Capture")
    cap = cv2.VideoCapture(camSet)
    target_x = 320
    target_y = 240
    dt = 0.1
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    sentroid_tag = {}

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Perform AprilTag detection
        result_frame, center_x, center_y, tag_id, tag_results = detect_apriltag(frame)
        cv2.line(frame, (310, 240), (330, 240), (0, 255, 255), 2)  # horizontal line (320,240)
        cv2.line(frame, (320, 230), (320, 250), (0, 255, 255), 2)  # vertical line
        cv2.line(frame, (0, 240), (640, 240), (0, 255, 255), 2)
        cv2.rectangle(frame, (285, 205), (355, 275), (0, 255, 255), 2)

        # Iterasi melalui hasil deteksi AprilTag
        for tag_result in tag_results:
            center_x = tag_result['center_x']
            center_y = tag_result['center_y']
            tag_id = tag_result['tag_id']
            distance = tag_result['distance']

            # Perbarui sentroid_tag dictionary
            sentroid_tag[tag_id] = (center_x, center_y)

        # Tampilkan hasil deteksi
        resizedFrame = cv2.resize(result_frame, (400, 400))
        cv2.imshow('AprilTag Detection', resizedFrame)

        # Print center coordinates
        for current_tag_id in range(len(sentroid_tag)):
            if current_tag_id in sentroid_tag:
                center_x, center_y = sentroid_tag[current_tag_id]
                x = 320 - center_x
                y = 240 - center_y
                px_x = px_meter(60, 640, alt=1.5)
                px_y = px_meter(60, 480, alt=1.5)
                x = x * 0.1 * px_x
                y = y * 0.1 * px_y
                vx = y
                vy = x
                vz = 0

				
		if current_tag_id == 0 and 300 <= center_x <= 340 and 220 <= center_y <= 260:
              
# Tambahkan tindakan atau perpindahan ke ID tag berikutnya
                    current_tag_id = 1

                elif current_tag_id == 1 and 300 <= center_x <= 340 and 220 <= center_y <= 260:
                    # Tindakan atau logika untuk ID tag 1
                    
                    # Tambahkan tindakan atau perpindahan ke ID tag berikutnya
                    current_tag_id = 2

                elif current_tag_id == 2 and 300 <= center_x <= 340 and 220 <= center_y <= 260:
                    # Tindakan atau logika untuk ID tag 2
                    
                    # Tambahkan tindakan atau perpindahan ke ID tag berikutnya
                    current_tag_id = 3

                elif current_tag_id == 3 and 300 <= center_x <= 340 and 220 <= center_y <= 260:
                    # Tindakan atau logika untuk ID tag 3
                    
                    # Tambahkan tindakan atau perpindahan ke ID tag berikutnya
                    current_tag_id = 4

                elif current_tag_id == 4 and 300 <= center_x <= 340 and 220 <= center_y <= 260:
                    # Tindakan atau logika untuk ID tag 4
                    
                    # Tambahkan tindakan atau perpindahan ke ID tag berikutnya
                    current_tag_id = 5

                print("vx =", str(vx))
                print("vy =", str(vy))
                print("vz =", str(vz))
                print("x =", center_x)
                print("y =", center_y)
                mav.move_drone(vx, vy, vz)

        # Exit jika tombol 'q' ditekan
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Bebaskan kamera dan tutup jendela OpenCV
    cap.release()
    cv2.destroyAllWindows()


def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    c = MavController2()
    rospy.sleep(4)

    print("Takeoff")
    c.takeoff(1.5)
    rospy.sleep(15)
    
    print("victim1")
    c.goto_xy (-1.3,0)
    rospy.sleep(10)
    
    print("wp 1")	
    c.goto_xy (1.3,0)
    rospy.sleep(10)
    
    print("wp 2")	
    c.goto_xy (0.0,2.25)
    rospy.sleep(10)
    
    print("victim2")
    c.gotoc_xy (2.5,2.3)
    rospy.sleep(20)
    
    print("victim3")
    c.gotoc_xy (10.0,1.7)
    rospy.sleep(35)
    
    print("waypoint2")
    c.gotoc_xy (6.5,4.4)
    rospy.sleep(25)
    
    print("waypoint3")
    c.gotoc_xy (5.6,7.35)
    rospy.sleep(15)
    
    print("victim3")
    c.gotoc_xy (10.8,1.1)
    rospy.sleep(25)
    
    print("waypoint3")
    c.gotoc_xy (5.6,7.35)
    rospy.sleep(15)
    
    print("waypoint2")
    c.gotoc_xy (6.5,4.4)
    rospy.sleep(20)
    
    print("wp 2")	
    c.gotoc_xy (0.0,2.25)
    rospy.sleep(10)
    
    """
    c.goto_xyz_rpy(0.0,0.0)
    rospy.sleep(20)
    print("coor x = 2 , y = 0")
    c.goto_xy (2,0)
    rospy.sleep(10)
    print("coor x = 2 , y = 0 (victim 3)")
    c.goto_xy (2,0)
    rospy.sleep(10)
    print("coor x = 2 , y = 0")
    c.goto_xy (2,0)
    rospy.sleep(10)
    print("coor x = 2 , y = 0")
    c.goto_xy (2,0)
    rospy.sleep(10)
    print("coor x = 2 , y = 0 (resupply area)")
    c.goto_xy (2,0)
    rospy.sleep(10)
    print ("x= 0 , y = 0 (victim 4)")
    c.goto_xyz_rpy(0.0,0.0)
    rospy.sleep(20)
    print ("x= 0 , y = 0")
    c.goto_xyz_rpy(0.0,0.0)
    rospy.sleep(20)
    print("coor x = 2 , y = 0 (resupply area)")
    c.goto_xy (2,0)
    rospy.sleep(10)
    print("coor x = 2 , y = 0 (victim 5)")
    c.goto_xy (2,0)
    rospy.sleep(10)

    print("Landing")
    c.land()
'''
def save_coor():
    #rospy.init_node('coordinator_node', anonymous=True)
    c1 = MavController()

    while not rospy.is_shutdown():
        current_pose = c1.get_current_pose()
        print("Current Pose: X={}, Y={}, Z={}".format(current_pose.position.x, current_pose.position.y, current_pose.position.z))

        # Adjust the delay (e.g., rospy.Rate) if needed
        rospy.sleep(1.0)  # 1 second delay
'''

if _name_ == "_main_":
        #uji1()
	simple_demo()
	#save_coor()
