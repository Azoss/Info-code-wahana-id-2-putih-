#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

PROJECT_ROOT_PATH = "catkin_ws/src/vision_to_mavros/scripts/apriltag_v1.py"
import sys
import time
import rospy
import tf
import cv2
import numpy as np
import apriltag
import math
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, TwistStamped
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL

from mavros_msgs.msg import PositionTarget

pi_2 = 3.141592654 / 2.0
sys.path.append(PROJECT_ROOT_PATH)

class MavController:
        """
        A simple object to help interface with mavros
        """
        def __init__(self):
                rospy.init_node("mav_control_node")
                rospy.Subscriber("/copter2/local_position/pose", PoseStamped, self.pose_callback)
                rospy.Subscriber("/copter2/rc/in", RCIn, self.rc_callback)
                self.pub = rospy.Publisher("/copter2/setpoint_raw/local", PositionTarget, queue_size=10)
                self.cmd_pos_pub = rospy.Publisher("/copter2/setpoint_position/local", PoseStamped, queue_size=1)
                self.cmd_vel_pub = rospy.Publisher("/copter2/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
                self.rc_override = rospy.Publisher("/copter2/rc/override", OverrideRCIn, queue_size=1)
                self.pub1 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

                # mode 0 = STABILIZE
                # mode 4 = GUIDED
                # mode 9 = LAND
                self.mode_service = rospy.ServiceProxy('/copter2/set_mode', SetMode)
                self.arm_service = rospy.ServiceProxy('/copter2/cmd/arming', CommandBool)
                self.takeoff_service = rospy.ServiceProxy('/copter2/cmd/takeoff', CommandTOL)

                self.rc = RCIn()
                self.pose = Pose()
                self.timestamp = rospy.Time()

        def rc_callback(self, data):
                """
                Keep track of the current manual RC values
                """
                self.rc = data

        def pose_callback(self, data):
                """
                Handle local position information
                """
                self.timestamp = data.header.stamp
                self.pose = data.pose

	def get_current_pose(self):
        	return self.pose

        def goto(self, pose):
                """
                Set the given pose as a the next setpoint by sending
                a SET_POSITION_TARGET_LOCAL_NED message. The copter must
                be in GUIDED mode for this to work.
                """
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.timestamp
                pose_stamped.pose = pose

                self.cmd_pos_pub.publish(pose_stamped)

	def goto_xy(self, x , y):
		pose = Pose()
		current_pose = self.get_current_pose()  # Mendapatkan pose saat ini
		mx = x
		my = y

		pose.position.x = current_pose.position.x + mx 
		pose.position.y = current_pose.position.y + my
		pose.position.z = current_pose.position.z

		'''quat = tf.transformations.quaternion_from_euler(0, 0, 0)'''

		pose.orientation.x = current_pose.orientation.x
		pose.orientation.y = current_pose.orientation.y
		pose.orientation.z = current_pose.orientation.z
		pose.orientation.w = current_pose.orientation.w
		self.goto(pose)

        def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z

                quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                self.goto(pose)
                #print(quat)

        def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
                """
                Send comand velocities. Must be in GUIDED mode. Assumes angular
                velocities are zero by default.
                """
                cmd_vel = Twist()

                cmd_vel.linear.x = vx
                cmd_vel.linear.y = vy
                cmd_vel.linear.z = vz

                cmd_vel.angular.x = avx
                cmd_vel.angular.y = avy
                cmd_vel.angular.z = avz

                self.cmd_vel_pub.publish(cmd_vel)
        def set_vel_move(self, vx, vy, vz):
                """
                Send comand velocities. Must be in GUIDED mode. Assumes angular
                velocities are zero by default.
                """
                cmd_vel = Twist()

                cmd_vel.linear.x = vx
                cmd_vel.linear.y = vy
                cmd_vel.linear.z = vz
                self.cmd_vel_pub.publish(cmd_vel)

        def move_drone(self, vx, vy, vz):
                msg = PositionTarget()
                msg.header.stamp = self.timestamp
                msg.header.frame_id = "frame"
                msg.coordinate_frame = 8
                msg.type_mask=4039
                msg.velocity.x = vx
                msg.velocity.y = vy
                msg.velocity.z = vz
                self.pub.publish(msg)

	def move_drone1(self, velx, vely, velz, duration):
            end_time = rospy.Time.now() + rospy.Duration(duration)
            while rospy.Time.now() < end_time and not       rospy.is_shutdown():
                msg = PositionTarget()
                #msg.header.stamp = self.timestamp
                msg.header.frame_id = "frame"
                msg.coordinate_frame = 8  # FRAME_BODY_OFFSET_NED pilihan 8/1
                msg.type_mask = 4039  # ignore orientation and thrust pilihan 4039 atau 4095
                msg.velocity.x = velx
                msg.velocity.y = vely
                msg.velocity.z = velz
                self.pub.publish(msg)
                
        def move_drone2(self, velx, vely, velz, duration):
            end_time = rospy.Time.now() + rospy.Duration(duration)
            while rospy.Time.now() < end_time and not       rospy.is_shutdown():
                msg = TwistStamped()
                msg.header.stamp = self.timestamp
                msg.header.frame_id = "frame"
                msg.coordinate_frame = 8  # FRAME_BODY_OFFSET_NED pilihan 8/1
                msg.type_mask = 4039  # ignore orientation and thrust pilihan 4039 atau 4095
                msg.velocity.x = velx
                msg.velocity.y = vely
                msg.velocity.z = velz
                self.pub.publish(msg)
                
        def move_drone3(self, velx, vely, velz, duration):
            rate = rospy.Rate(5)  # Frekuensi pengiriman pesan
            start_time = rospy.Time.now()
            end_time = rospy.Time.now() + rospy.Duration(duration)
            while rospy.Time.now() < end_time and not       rospy.is_shutdown():
                msg = PositionTarget()
                msg.header.stamp = self.timestamp
                msg.header.frame_id = "frame"
                msg.coordinate_frame = 8  # FRAME_BODY_OFFSET_NED pilihan 8/1
                msg.type_mask = 4039  # ignore orientation and thrust pilihan 4039 atau 4095
                msg.velocity.x = velx
                msg.velocity.y = vely
                msg.velocity.z = velz
                self.pub.publish(msg)
                
        def send_velocity(self, velocity_x, velocity_y, velocity_z, duration):
            rate = rospy.Rate(10)  #Frekuensi pengiriman pesan (10 Hz)
            start_time = rospy.Time.now()
            end_time = start_time + rospy.Duration(duration) 
            while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                cmd_vel = TwistStamped()
                cmd_vel.header.stamp = rospy.Time.now()
                cmd_vel.twist.linear.x = velocity_x
                cmd_vel.twist.linear.y = velocity_y
                cmd_vel.twist.linear.z = velocity_z
                self.pub1.publish(cmd_vel)
                rate.sleep() 
                
        def arm(self):
                """
                Arm the throttle
                """
                return self.arm_service(True)

        def disarm(self):
                """
                Disarm the throttle
                """
                return self.arm_service(False)

        def takeoff(self, height=1.0):
                """
                Arm the throttle, takeoff to a few feet, and set to guided mode
                """
                # Set to stabilize mode for arming
                #mode_resp = self.mode_service(custom_mode="0")
                mode_resp = self.mode_service(custom_mode="4")
                self.arm()

                # Set to guided mode
                #mode_resp = self.mode_service(custom_mode="4")

                # Takeoff
                takeoff_resp = self.takeoff_service(altitude=height)

                #return takeoff_resp
                return mode_resp

        def land(self):
                """
                Set in LAND mode, which should cause the UAV to descend directly,
                land, and disarm.
                """
                resp = self.mode_service(custom_mode="9")
                self.disarm()

class PIDController:
        def __init__(self, Kp, Ki, Kd):
                self.Kp = Kp
                self.Ki = Ki
                self.Kd = Kd
                self.prev_error = 0
                self.integral = 0
        def reset_integral(self):
                self.integral = 0
        def calculate(self, error, dt):
                self.integral += error * dt
                derivative = (error - self.prev_error) / dt
                output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
                self.prev_error = error
                return output

def px_meter (fov, res, alt):
        return ((alt * math.tan(math.radians(fov/2))) / (res/2))
def detect_apriltag1(image):
    detector = apriltag.get_detector()
    #detector = apriltag.Detector()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Detect AprilTags in the image
    detections, dimg = detector.detect(gray, return_image=True)

    center_x, center_y, tag_id = None, None, None
    tag_results = []

    # Extract corner coordinates from all detections
    all_corners = [detection.corners.astype(int) for detection in detections]

    # Draw a single bounding box that encompasses all detections
    if len(all_corners) > 0:
        all_corners = np.concatenate(all_corners, axis=0)
        min_x, min_y = np.min(all_corners, axis=0)
        max_x, max_y = np.max(all_corners, axis=0)
        cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 0), 2)

        # Draw center of the bounding box
        center_x = int((min_x + max_x) / 2)
        center_y = int((min_y + max_y) / 2)
        cv2.circle(image, (center_x, center_y), 2, (0, 0, 255), -1)
        cv2.line(image, (center_x, center_y), (320, 240), (0, 255, 255), 2)

        # Draw ID and distance for the first detection
        if len(detections) > 0:
            detection = detections[0]
            tag_id = detection.tag_id
            a = "ID: {}".format(tag_id)
            cv2.putText(image, a, (min_x, min_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
            tag_size = 0.16  # Example tag size in meters (adjust according to your tag)
            focal_length = 800  # Example focal length in pixels (adjust according to your camera)
            distance = tag_size * focal_length / detection.homography[2, 2]
            cv2.putText(image, str(distance), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
            tag_results.append({
                'center_x': center_x,
                'center_y': center_y,
                'tag_id': tag_id,
                'distance': distance
            })

    return image, center_x, center_y, tag_id, tag_results
    
def detect_apriltag(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)


    # Use the aruco module for AprilTag detection
    #aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    #parameters = cv2.aruco.DetectorParameters_create()

    # Detect markers in the image
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    center_x, center_y, tag_id = None, None, None
    tag_results = []

    # Draw markers and calculate center coordinates
    if ids is not None:
        for i in range(len(ids)):
            marker_id = ids[i][0]
            corners_i = corners[i].reshape(-1, 2).astype(np.int32)

            # Draw the marker
            cv2.polylines(image, [corners_i], isClosed=True, color=(0, 255, 0), thickness=2)
            center_x = int(np.mean(corners_i[:, 0]))
            center_y = int(np.mean(corners_i[:, 1]))

            # Draw center of the marker
            cv2.circle(image, (center_x, center_y), 2, (0, 0, 255), -1)
            cv2.line(image, (center_x, center_y), (320, 240), (0, 255, 255), 2)

            # Draw ID for the marker
            a = "ID: {}".format(marker_id)
            cv2.putText(image, a, (corners_i[0, 0], corners_i[0, 1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                        cv2.LINE_AA)

            tag_results.append({
                'center_x': center_x,
                'center_y': center_y,
                'tag_id': marker_id,
                'distance': 0  # You may need to compute distance based on your setup
            })

    return image, center_x, center_y, tag_id, tag_results
'''
def detect_apriltag(image):
        #options = apriltag.DetectorOptions(families='tag36h11')
		#options = {'families': 'tag36h11'}
	#detector = apriltag.Detector(options=options)
        #detector = apriltag.Detector(options)
		detector = apriltag.Detector()

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags in the image
        detections, dimg = detector.detect(gray, return_image=True)

        center_x, center_y, tag_id= None, None, None  # Inisialisasi variabel x dan y
        tag_results = []
        # Draw detection results on the image
        if len(detections) > 0:
                for detection in detections:
                        # Extract corner coordinates
                        corners = detection.corners.astype(int)

                        # Draw bounding box
                        cv2.polylines(image, [corners], True, (0, 255, 0), 2)

                        # Draw center
                        center = np.mean(corners, axis=0, dtype=int).flatten()
                        #center_x, center_y = center  # Simpan koordinat pusat
                        center_x = int(np.mean(corners[:, 0]))
                        center_y = int(np.mean(corners[:, 1]))
                        cv2.circle(image, tuple(center), 2, (0, 0, 255), -1)
                        cv2.line(image, (center_x,center_y),(320,240),(0,255,255),2)
                        # Draw ID and distance
                        tag_id = detection.tag_id
                        a =  "ID: {}".format(tag_id)
                        cv2.putText(image, a, (corners[0][0], corners[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                        # Calculate distance (assuming AprilTag size is known)
                        tag_size = 0.16  # Example tag size in meters (adjust according to your tag)
                        focal_length = 800  # Example focal length in pixels (adjust according to your camera)
                        distance = tag_size * focal_length / detection.homography[2, 2]
                        
                        # Draw distance
                        cv2.putText(image, str(distance), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
                        tag_results.append({
                'center_x': center_x,
                'center_y': center_y,
                'tag_id': tag_id,
                'distance': distance
            })
        return image, center_x, center_y, tag_id, tag_results
'''
def main():
        # Open webcam
	camSet='nvarguscamerasrc sensor-is=0 ! video/x-raw(memory:NVMM), width=640, 		height=480,framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

        mav = MavController()
        rospy.sleep(4)
        Kp_x = Kp_y = 0.03
        Ki_x = Ki_y = 0.03
        Kd_x = Kd_y = 0.03
        pid_x = PIDController(Kp_x, Ki_x, Kd_x)
        pid_y = PIDController(Kp_y, Ki_y, Kd_y)
        
	print("capture")
	#mav.move_drone1(0.08,0,0,5)
        #cap = cv2.VideoCapture(camSet)
	cap = cv2.VideoCapture(0)
        target_x = 320
        target_y = 240
        dt = 0.1
        
        print("TakeOff")
        mav.takeoff(1.5)
        rospy.sleep(15)
        
        print("Maju")
        mav.move_drone1(0.1, 0, 0, 10)
        
        if not cap.isOpened():
                print("Error: Could not open webcam.")
                return
	
        while True:
                # Read a frame from the webcam
                ret, frame = cap.read()

                if not ret:
                        print("Error: Failed to capture frame.")
                        break

                # Perform AprilTag detection
                result_frame, center_x, center_y, tag_id, tag_results = detect_apriltag(frame)
                cv2.line(frame, (310,240), (330,240), (0,255,255),2) #horizontal line (320,240)
                cv2.line(frame, (320,230), (320,250), (0,255,255),2) #vertical line
                cv2.line(frame, (0,240), (640,240), (0,255,255),2)
                cv2.rectangle(frame, (280,200),(360,280),(0,255,255),2)

                
                for tag_result in tag_results:
                        center_x = tag_result['center_x']
                        center_y = tag_result['center_y']
                        tag_id = tag_result['tag_id']
                        distance = tag_result['distance']
                        
                if tag_id == 3:
                    #skip sek lur
                        continue
                            
                #if tag_id == 0 and 300 <= center_x <= 340 and 220 <= center_y <= 260:
                        #print("awww")

                # Display the result
		resizedFrame = cv2.resize(result_frame, (400, 400))
                cv2.imshow('AprilTag Detection', resizedFrame)


                # Print center coordinates
                if center_x is not None and center_y is not None:
                        kx = center_x 
                        ky = center_y 
                        error_x = target_x - center_x
                        error_y = target_y - center_y
                        pid_x.reset_integral()
                        pid_y.reset_integral()
                        #x = 320 - center_x 
                        #y = 240 - center_y 
                        px_x = px_meter(60, 640, alt = 1.5)
                        px_y = px_meter(60, 480, alt = 1.5)
                        #x = x * 0.1 * px_x
                        #y = y * 0.1  * px_y 
                        vy = pid_x.calculate(error_x, dt)
                        vx = pid_y.calculate(error_y, dt)
                        vz = 0
                        vy *= px_x
                        vx *= px_y
                        #vx = y
                        #vy = x 
                        #vz = 0
                        
                        if tag_id == 1 and kx > 280 and kx < 360 and ky > 200 and ky < 280:
                                print("tagid 11111111111111111111")
                                vx = vy = 0
                                mav.move_drone1(0.15,0,0,7)
                                time.sleep(0.5)
                                
                        elif tag_id == 2 and kx > 280 and kx < 360 and ky > 200 and ky < 280:
                                print("tagid 3333333333333333333")
                                vx = vy = 0
                                mav.goto_xy(0, 0.8)
                                time.sleep(0.5)
                                
                        print ("vx = " + str(vx))
                        print("vy=" + str(vy))
                        print ("vz=" + str(vz))
                        print("x=" + str(kx))
                        print("y=" +str(ky))
                        print("id=", tag_id)
                        print("x=", center_x)
                        print("y=", center_y)
                        #mav.set_vel_move(vx, vy, vz)
                        mav.move_drone(vx, vy, vz)
                        #rospy.sleep(0.5)
                # Exit when 'q' key is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
	# Release the webcam and close all windows
	cap.release()
	cv2.destroyAllWindows()
        
def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    c = MavController()
    rospy.sleep(1)

    print("Takeoff")
    c.takeoff(0.5)
    rospy.sleep(3)
    c.goto_xyz_rpy(0,0,1.2,0,0,0)
    rospy.sleep(3)

    print("Waypoint 1: position control")
    c.goto_xyz_rpy(0.0,0.0,1.2,0,0,-1*pi_2)
    rospy.sleep(2)
    c.goto_xyz_rpy(0.4,0.0,1.2,0,0,-1*pi_2)
    rospy.sleep(3)
    print("Waypoint 2: position control")
    c.goto_xyz_rpy(0.4,0.0,1.2,0,0,0)
    rospy.sleep(2)
    c.goto_xyz_rpy(0.4,0.4,1.2,0,0,0)
    rospy.sleep(3)
    print("Waypoint 3: position control")
    c.goto_xyz_rpy(0.4,0.4,1.2,0,0,pi_2)
    rospy.sleep(2)
    c.goto_xyz_rpy(0.0,0.4,1.2,0,0,pi_2)
    rospy.sleep(3)
    print("Waypoint 4: position control")
    c.goto_xyz_rpy(0.0,0.4,1.2,0,0,2*pi_2)
    rospy.sleep(2)
    c.goto_xyz_rpy(0.0,0.0,1.2,0,0,2*pi_2)
    rospy.sleep(3)

    #print("Velocity Setpoint 1")
    #c.set_vel(0,0.1,0)
    #rospy.sleep(5)
    #print("Velocity Setpoint 2")
    #c.set_vel(0,-0.1,0)
    #rospy.sleep(5)
    #print("Velocity Setpoint 3")
    #c.set_vel(0,0,0)
    #rospy.sleep(5)

    print("Landing")
    c.land()


import cv2
import apriltag

def detect_apriltag_realtime():
    # Inisialisasi detektor AprilTag
    detector = apriltag.Detector()
    camSet = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=640, height=480, framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

    # Inisialisasi kamera (ganti 0 dengan indeks kamera jika menggunakan kamera eksternal)
    cap = cv2.VideoCapture(camSet)

    while True:
        # Baca frame dari kamera
        ret, frame = cap.read()

        # Ubah ke skala abu-abu
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Deteksi AprilTag
        detections, dimg = detector.detect(gray, return_image=True)

        # Loop melalui deteksi dan tampilkan hasilnya
        for i, detection in enumerate(detections):
            # Dapatkan pusat AprilTag
            center_x = int(detection.center[0])
            center_y = int(detection.center[1])

            print("Detected AprilTag {i + 1}:")
            print("ID: {detection.tag_id}")
            print("Center: ({center_x}, {center_y})")

            # Gambar kotak deteksi secara manual
            rect_points = detection.corners.astype(int)
            cv2.polylines(frame, [rect_points], isClosed=True, color=(0, 255, 0), thickness=2)

        # Tampilkan gambar dengan kotak deteksi
        cv2.imshow("AprilTag Detection", frame)

        # Hentikan loop jika tombol 'q' ditekan
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Tutup kamera dan jendela OpenCV
    cap.release()
    cv2.destroyAllWindows()


def main2():
        # Open webcam
	camSet='nvarguscamerasrc sensor-is=0 ! video/x-raw(memory:NVMM), width=640, 		height=480,framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=800, height=600, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

        mav = MavController()
        rospy.sleep(4)
        Kp_x = Kp_y = 0.07
        Ki_x = Ki_y = 0.07
        Kd_x = Kd_y = 0.07
        pid_x = PIDController(Kp_x, Ki_x, Kd_x)
        pid_y = PIDController(Kp_y, Ki_y, Kd_y)
        mav.takeoff(1.5)
        rospy.sleep(10)
	print("capture")
        #cap = cv2.VideoCapture(camSet)
	cap = cv2.VideoCapture(0)
        target_x = 320
        target_y = 240
        dt = 0.1
        if not cap.isOpened():
                print("Error: Could not open webcam.")
                return
	
        while True:
                # Read a frame from the webcam
                ret, frame = cap.read()

                if not ret:
                        print("Error: Failed to capture frame.")
                        break

                # Perform AprilTag detection
                result_frame, center_x, center_y, tag_id, tag_results = detect_apriltag(frame)
                cv2.line(frame, (310,240), (330,240), (0,255,255),2) #horizontal line (320,240)
                cv2.line(frame, (320,230), (320,250), (0,255,255),2) #vertical line
                cv2.line(frame, (0,240), (640,240), (0,255,255),2)
                cv2.rectangle(frame, (300,220),(340,260),(0,255,255),2)
                
                for tag_result in tag_results:
                        center_x = tag_result['center_x']
                        center_y = tag_result['center_y']
                        tag_id = tag_result['tag_id']
                        distance = tag_result['distance']
                        
                        if tag_id == 3:
                            #skip sek lur
                            continue
		
		        # Display the result
			#resizedFrame = cv2.resize(result_frame, (640, 680))
		        cv2.imshow('AprilTag Detection', frame)

		        # Print center coordinates
		        if center_x is not None and center_y is not None:
		                kx = center_x 
		                ky = center_y 
		                error_x = target_x - center_x
		                error_y = target_y - center_y
		                pid_x.reset_integral()
		                pid_y.reset_integral()
		                #x = 320 - center_x 
		                #y = 240 - center_y 
		                px_x = px_meter(60, 640, alt = 1.5)
		                px_y = px_meter(60, 480, alt = 1.5)
		                #x = x * 0.1 * px_x
		                #y = y * 0.1  * px_y 
		                vy = pid_x.calculate(error_x, dt)
		                vx = pid_y.calculate(error_y, dt)
		                vz = 0
		                vy *= px_x
		                vx *= px_y
		                #vx = y
		                #vy = x 
		                #vz = 0
		                
		                if tag_id == 1 and kx > 280 and kx < 360 and ky > 200 and ky < 280:
		                        print("tagid 11111111111111111111")
		                        vx = vy = 0
		                        mav.move_drone1(0.15,0,0,7)
		                        time.sleep(0.5)
		                        
		                elif tag_id == 2 and kx > 280 and kx < 360 and ky > 200 and ky < 280:
		                        print("tagid 3333333333333333333")
		                        vx = vy = 0
		                        mav.goto_xy(0, 0.8)
		                        time.sleep(0.5)
		                                
		                print ("vx = " + str(vx))
		                print("vy=" + str(vy))
		                print ("vz=" + str(vz))
		                print("x=" + str(kx))
		                print("y=" +str(ky))
		                print("id=", tag_id)
		                print("x=", center_x)
		                print("y=", center_y)
		                #mav.set_vel_move(vx, vy, vz)
		                mav.move_drone(vx, vy, vz)

                # Exit when 'q' key is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
	# Release the webcam and close all windows
	cap.release()
	cv2.destroyAllWindows()
 
def takeoff():
    c = MavController()
    rospy.sleep(4)

    print("Take Off 1,5M")
    c.takeoff(1.5)
    rospy.sleep(15)
    
    print("Move Drone1 X = 0.1 Duration 15")
    c.send_velocity(0.1, 0, 0, 10)
    #rospy.sleep(10)

    print("land")
    c.land()
if __name__ == "__main__":
        #detect_apriltag_realtime()
        #main()
        takeoff()