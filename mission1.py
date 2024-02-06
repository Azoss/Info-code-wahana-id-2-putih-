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

def simple_demo():
    """
    A simple demonstration of using mavros commands to control a UAV.
    """
    
    x1 = 0
    y1 = 0.9*2.5
    sleep1 = 9
    x2 = 0.9*2.5
    y2 = 0
    sleep2 = 9
    c = MavController2()
    rospy.sleep(5)
    print("TakeOff")
    c.takeoff(2.5)
    rospy.sleep(20)
    
    print("+1y 2.5 meter")
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)
    
    print("+1y 5 meter")
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)
    
    print("+1y 7.5 meter")
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)
    
    print("+1y 10 meter")
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)
    
    print("+1y 11.5 meter")#setengah
    c.goto_xy (0,1.35)
    rospy.sleep(6)
    
    print("+1x 1 meter")
    c.goto_xy (x2,y2)
    rospy.sleep(sleep2)
    
    print("+1x 2 meter")
    c.goto_xy (x2,y2)
    rospy.sleep(sleep2)
    
    print("+1x 3 meter")
    c.goto_xy (x2,y2)
    rospy.sleep(sleep2)
    
    print("+1x 4 meter")
    c.goto_xy (x2,y2)
    rospy.sleep(sleep2)
    
    print("+1x 5 meter")
    c.goto_xy (x2,y2)
    rospy.sleep(sleep2)
    
    print("+1x 13.5 meter")#tengah2
    c.goto_xy (0.9,0)
    rospy.sleep(5)
    
    print("Landing")
    c.land()
    
def takeoff():
    c = MavController2()
    rospy.sleep(5)
    print("TakeOff")
    c.takeoff(1)
    rospy.sleep(25)
    
    #print("coor x = 0 , y = 9.7")
    #c.goto_xy (0,9.7)
    #rospy.sleep(40)
    
    print("Landing")
    c.land()
   

def victim1():
    #simpan variable
    #menuju ke resupply area (+y 9.5m ke resuppy area)
    #8 meters
    x1 = 0
    y1 = 0.9*4
    sleep1 = 13
    #1.5 meters
    x1s = 0
    y1s = 0.9*1.5
    sleep1s = 6

    #menuju ke victim1 area (-y 5.3m ke victim area)
    x2 = 0
    y2 = -0.9*4
    sleep2 = 13
    #1.2 meters
    x2s = 0
    y2s = -0.9*1.4
    sleep2s = 5
    #menuju ke victim1 area (+x 12m ke victim area)
    x3 = 0.9*4
    y3 = 0
    sleep3 = 13

    #balikke home (-x 12m ke home area)
    x4 = -0.9*4
    y4 = 0
    sleep4 = 13
    #balikke home (-y 3.8m ke home area)
    x5 = 0
    y5 = -0.9*3.8
    sleep5 = 12

    c = MavController2()
    rospy.sleep(5)
    print("TakeOff")
    c.takeoff(1.5)
    rospy.sleep(15)
    
    #ke resupply
    print("Y 4 meter ke resupply")
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)

    print("Y 8 meter ke resupply")
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)

    print("Y 9.5 meter ke resupply")
    c.goto_xy (x1s,y1s)
    rospy.sleep(sleep1s)

    #ke victim
    print("-Y 4 meter ke victim")
    c.goto_xy (x2,y2)
    rospy.sleep(sleep2)

    print("-Y 5.2 meter ke victim")
    c.goto_xy (x2s,y2s)
    rospy.sleep(sleep2s)

    print("X 4 meter ke victim")
    c.goto_xy (x3,y3)
    rospy.sleep(sleep3)

    print("X 8 meter ke victim")
    c.goto_xy (x3,y3)
    rospy.sleep(sleep3)

    print("X 12 meter ke victim")
    c.goto_xy (x3,y3)
    rospy.sleep(sleep3)

    #ke home
    print("-X 4 meter ke home")
    c.goto_xy (x4,y4)
    rospy.sleep(sleep4)

    print("-X 8 meter ke home")
    c.goto_xy (x4,y4)
    rospy.sleep(sleep4)

    print("-X 12 meter ke home")
    c.goto_xy (x4,y4)
    rospy.sleep(sleep4)

    print("-X 3.8 meter ke home")
    c.goto_xy (x5,y5)
    rospy.sleep(sleep5)

    print("Landing")
    c.land()


def victim2():
    #simpan variable
    #menuju ke resupply area (+y 9.5m ke resuppy area)
    #8 meters
    x1 = 0
    y1 = 0.9*4
    sleep1 = 13
    #1.5 meters
    x1s = 0
    y1s = 0.9*1.5
    sleep1s = 5
    
    #menuju ke victim2 area (-y 5.2m ke victim area)
    x2 = 0
    y2 = -0.9*4
    sleep2 = 13
    #1.2 meters
    x2s = 0
    y2s = -0.9
    sleep2s = 4
    
    #menuju ke victim2 area (+x 8m ke victim area)
    x3 = 0.9*4
    y3 = 0
    sleep3 = 13
    
    c = MavController2()
    rospy.sleep(5)
    print("TakeOff")
    c.takeoff(2.5)
    rospy.sleep(20)
    
    #ke resupply
    print("Y 4 meter ke resupply")
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)

    print("Y 8 meter ke resupply")
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)

    print("Y 9.5 meter ke resupply")
    c.goto_xy (x1s,y1s)
    rospy.sleep(sleep1s)
    
    #ke victim
    print("-Y 4 meter ke victim")
    c.goto_xy (x2,y2)
    rospy.sleep(sleep2)

    print("-Y 5 meter ke victim")
    c.goto_xy (x2s,y2s)
    rospy.sleep(sleep2s)
    
    print("X 4 meter ke victim") 
    c.goto_xy (x3,y3)
    rospy.sleep(sleep3)

    print("X 8 meter ke victim")
    c.goto_xy (x3,y3)
    rospy.sleep(sleep3)
    
def takeoff():
    '''
    y1 = 0.8*3
    x1 = 0
    sleep1 = 10
    '''
    c = MavController2()
    rospy.sleep(5)
    print("TakeOff")
    
    c.takeoff(1.5)
    rospy.sleep(15)
    
    print("Move 1X")
    c.move_drone1(0.1, 0.0, 0.0, 10)
    
    print("Move 1Y")
    c.move_drone1(0.0, 0.1, 0.0, 10)
    
    #print("Move 1X")
    #c.move_drone1(0.0, 0.0,, 10)
    '''
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)
    
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)
    
    c.goto_xy (x1,y1)
    rospy.sleep(sleep1)
    '''
    print("land")

def uji_rangefinder():

    c = MavController2()
    rospy.sleep(5)
    print("TakeOff")

    c.takeoff(1.5)
    rospy.sleep(20)
    '''
    print("lewat balok")
    c.goto_balok(4.0,0,1.5)
    rospy.sleep(24)
    '''
    c.land()
if __name__ == "__main__":
    #uji1()
    #simple_demo()
    #save_coor()
    #takeoff()
    #victim1()
    uji_rangefinder()
