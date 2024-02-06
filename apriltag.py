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
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn, PositionTarget
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL

pi_2 = 3.141592654 / 2.0
        

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

        def move_drone_time(self, vx, vy, vz, duration):
                """
                Send command velocities for a specified duration. Must be in GUIDED mode.
                Assumes angular velocities are zero by default.
                """
                end_time = rospy.Time.now() + rospy.Duration(duration)
                while rospy.Time.now() < end_time and not rospy.is_shutdown():
                        msg = PositionTarget()
                        msg.header.stamp = self.timestamp
                        msg.header.frame_id = "frame"
                        msg.coordinate_frame = 8
                        msg.type_mask = 4039
                        msg.velocity.x = vx
                        msg.velocity.y = vy
                        msg.velocity.z = vz
                        self.pub.publish(msg)

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

def detect_apriltag(image):
	# Set detection options
	#options = apriltag.DetectorOptions(families='tag36h11')
	detector = apriltag.Detector()
	# Convert the image to grayscale
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
   	# Detect AprilTags in the image
	detections = detector.detect(gray)
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
			tag_size = 0.163  # Example tag size in meters (adjust according to your tag)
			focal_length = 15.1183007001  # Example focal length in pixels (adjust according to your camera) 4 mm c270
			distance = tag_size * focal_length / detection.homography[2, 2]
			# Draw distance
			cv2.putText(image, str(distance), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
			tag_results.append({
                'center_x': center_x,
                'center_y': center_y,
                'tag_id': tag_id,
                'distance': distance
            })
	return image, tag_results

def main():
        # Open webcam
        mav = MavController()
        rospy.sleep(4)
        Kp_x = Kp_y = 0.3
        Ki_x = Ki_y = 0.2
        Kd_x = Kd_y = 0.2
        pid_x = PIDController(Kp_x, Ki_x, Kd_x)
        pid_y = PIDController(Kp_y, Ki_y, Kd_y)
        mav.takeoff(1.5)
        rospy.sleep(4)
        cap = cv2.VideoCapture('/dev/video0')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        target_x = 320
        target_y = 240
        dt = 0.1
        mav.move_drone_time(0.5,0,0,3)
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
                result_frame, tag_results = detect_apriltag(frame)
                cv2.line(frame, (310,240), (330,240), (0,255,255),2) #horizontal line (320,240)
                cv2.line(frame, (320,230), (320,250), (0,255,255),2) #vertical line
                cv2.line(frame, (0,240), (640,240), (0,255,255),2)
                cv2.rectangle(frame, (300,220),(340,260),(0,255,255),2)
                selected_tag = None
                min_distance = float('inf')
                for tag_result in tag_results:
                        center_x = tag_result['center_x']
                        center_y = tag_result['center_y']
                        tag_id = tag_result['tag_id']
                        distance = tag_result['distance']
                        if distance < min_distance:
                                min_distance = distance
                                selected_tag = tag_result
                if selected_tag is not None:
                        if tag_id == 0 and 300 <= center_x <= 340 and 220 <= center_y <= 260:
                                print("awww")
                        

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
                                #x = x * 0.3 * px_x
                                #y = y * 0.3  * px_y 
                                vy = pid_x.calculate(error_x, dt)
                                vx = pid_y.calculate(error_y, dt)
                                #vz = 0
                                vy *= px_x
                                vx *= px_y
                                #vx = y
                                #vy = x 
                                vz = 0
                                
                                if kx  > 300 and kx < 340 :
                                        if ky > 220 and ky < 260 :
                                                mav.move_drone_time(0.2,0,0,3)
                                                print ("oke")
                                                
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
                                #rospy.sleep(0.1)
                # Display the result
                cv2.imshow('AprilTag Detection', result_frame)
                # Exit when 'q' key is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        # Release the webcam and close all windows
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
        main()
