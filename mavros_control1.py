#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
import tf
import cv2
import math
import numpy as np
import time
import threading
from time import sleep
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, TwistStamped
from mavros_msgs.msg import PositionTarget, OverrideRCIn, RCIn, HomePosition, ActuatorControl
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode, CommandLong
from nav_msgs.msg import Odometry
import os.path
import subprocess
import sys
pi = math.pi
pi_2 = pi / 2.0

class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)
	self.pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()
        self.pose = PoseStamped()

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

    def move_drone1(self, velx, vely, velz, duration):
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not         rospy.is_shutdown():
            msg = PositionTarget()
            msg.header.stamp = self.timestamp
            msg.header.frame_id = "frame"
            msg.coordinate_frame = 8  # FRAME_BODY_OFFSET_NED pilihan 8/1
            msg.type_mask = 4039  # ignore orientation and thrust pilihan 4039 atau 4095
            msg.velocity.x = velx
            msg.velocity.y = vely
            msg.velocity.z = velz
            self.pub.publish(msg)

    def move_drone(self, vx, vy, vz):
        # Create the NED velocity vector.
        #ned_vec = np.array([[self.vel_x], [self.vel_y], [self.vel_z]], dtype=np.float32)
        msg = PositionTarget()
        msg.header.stamp = self.timestamp
        msg.header.frame_id = "frame"
        msg.coordinate_frame = 8 # FRAME_BODY_OFFSET_NED pilihan 8/1
        msg.type_mask = 4039 # ignore orientation and thrust pilihan 4039 atau 4095
        #msg = np.array(velocity)
        msg.velocity.x = vx
        msg.velocity.y = -vy
        msg.velocity.z = vz
        self.pub.publish(msg)

def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    c = MavController()
    rospy.sleep(1)

    print("Takeoff")
    c.takeoff(1.0)
    rospy.sleep(7)
    while not rospy.is_shutdown():
	print("gerak")
    	c.move_drone(0.15,0,0)#x,y,z

    #print("Landing")
    #c.land()

if __name__=="__main__":
    simple_demo()
