#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import time
import rospy
import tf
import cv2
import numpy as np
import apriltag
import math
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, TwistStamped
from mavros_msgs.msg import OverrideRCIn, PositionTarget
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL

pi_2 = 3.141592654 / 2.0
        

class MavController2:
        """
        A simple object to help interface with mavros
        """
        def __init__(self):
                rospy.init_node("mav_control_node")
                rospy.Subscriber("/copter2/vision_pose/pose", PoseStamped, self.pose_callback)
                rospy.Subscriber("/copter2/rc/in", RCIn, self.rc_callback)
                self.pub = rospy.Publisher("/copter2/setpoint_raw/local", PositionTarget, queue_size=10)
                self.cmd_pos_pub = rospy.Publisher("/copter2/setpoint_position/local", PoseStamped, queue_size=1)
                self.cmd_vel_pub = rospy.Publisher("/copter2/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
                self.rc_override = rospy.Publisher("/copter2/rc/override", OverrideRCIn, queue_size=1)
		self.pub1 = rospy.Publisher('/copter2/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

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


	def goto_xyz(self, x , y, z):
                pose = Pose()
                current_pose = self.get_current_pose()  # Mendapatkan pose saat ini
                mx = x
                my = y

                pose.position.x = current_pose.position.x + mx
                pose.position.y = current_pose.position.y + my
                pose.position.z = current_pose.position.z + z

                '''quat = tf.transformations.quaternion_from_euler(0, 0, 0)'''

                pose.orientation.x = current_pose.orientation.x
                pose.orientation.y = current_pose.orientation.y
                pose.orientation.z = current_pose.orientation.z
                pose.orientation.w = current_pose.orientation.w
                self.goto(pose)

	
	def goto_balok(self, x , y, z):
                pose = Pose()
                current_pose = self.get_current_pose()  # Mendapatkan pose saat ini

                pose.position.x = current_pose.position.x + x
                pose.position.y = current_pose.position.y + y
                pose.position.z = z

                '''quat = tf.transformations.quaternion_from_euler(0, 0, 0)'''

                pose.orientation.x = current_pose.orientation.x
                pose.orientation.y = current_pose.orientation.y
                pose.orientation.z = current_pose.orientation.z
                pose.orientation.w = current_pose.orientation.w
                self.goto(pose)



        def goto_z(self, z):
		pose = Pose()
		current_pose = self.get_current_pose()  # Mendapatkan pose saat ini
		mz = z

		pose.position.x = current_pose.position.x 
		pose.position.y = current_pose.position.y 
		pose.position.z = current_pose.position.z + mz

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

	def send_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        	rate = rospy.Rate(10)  # Frekuensi pengiriman pesan (10 Hz)
        	start_time = rospy.Time.now()
        	end_time = start_time + rospy.Duration(duration) 
        	while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            		cmd_vel = TwistStamped()
            		cmd_vel.header.stamp = self.timestamp
			#cmd_vel.type_mask = 4039
            		cmd_vel.twist.linear.x = velocity_x
            		cmd_vel.twist.linear.y = velocity_y
            		cmd_vel.twist.linear.z = velocity_z
            		self.pub1.publish(cmd_vel)
            		rate.sleep()
        
	def move_drone1(self, velx, vely, velz, duration):
		rate = rospy.Rate(10)
            	start_time = rospy.Time.now()
            	while not rospy.is_shutdown():
 			waktu = rospy.Time.now()
			w =waktu - start_time
			if w.to_sec() >= duration:
				break
		
                	msg = PositionTarget()
               		msg.header.stamp = self.timestamp
                	msg.header.frame_id = "frame"
                	msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED # FRAME_BODY_OFFSET_NED pilihan 8/1
                	msg.type_mask = 4039  # ignore orientation and thrust pilihan 4039 atau 4095
                	msg.velocity.x = velx
                	msg.velocity.y = vely
                	msg.velocity.z = velz
                	self.pub.publish(msg)
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

