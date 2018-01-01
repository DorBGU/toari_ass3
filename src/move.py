#!/usr/bin/env python
import numpy as np
import ctypes
import struct
import rospy
import roslib
import time
import math
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, JointState, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import openni2_camera


rospy.init_node('robot_whisperer', anonymous=True)
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
OBSTACLE_DISTANCE_LIMIT = 0.5
start_travel_center_dist = -1
current_center_dist = -1
drive_flag = False
stable_flag = True
distance_to_red_object_flag = True
distance = float('inf')
here = True
depth_bool = True
x = -1
y = -1

## WHEN FOUND RED OBJECT, WHAT'S IT'S DISTANCE


def depth_callback(data):
    global distance, depth_bool
    depth_bool = False
    print "im in the depth"
    bridge = CvBridge()
    d_im = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    print d_im

    if x > -1:
        print d_im[y, x]
        distance = d_im[y, x]



def analyze_image(image):
    global x, y, depth_bool
    # print "start func"

    bridge = CvBridge()
    im = bridge.imgmsg_to_cv2(image, "bgr8")

    # red color boundaries (R,B and G)
    lower = [1, 0, 60]
    upper = [60, 40, 255]

    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    # change image to hsv and get the mask by lower and upper
    mask = cv2.inRange(im, lower, upper)
    output = cv2.bitwise_and(im, im, mask=mask)

    # cv2.imshow("Result", mask)
    # cv2.waitKey(33)

    et, thresh = cv2.threshold(mask, 40, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) != 0:
        # draw in blue the contours that we found
        cv2.drawContours(output, contours, -1, 255, 3)

        # find the biggest area
        c = max(contours, key=cv2.contourArea)
        ((cen_x, cen_y), rad) = cv2.minEnclosingCircle(c)

        if rad > 10:

            cv2.circle(im, (int(cen_x), int(cen_y)), int(rad), (0, 255, 0), 2)
            x = int(cen_x)
            y = int(cen_y)

            depth_bool = True
            sub = rospy.Subscriber("/torso_camera/depth_registered/image_raw", Image, depth_callback)
            # print "subs to depth"
            while depth_bool:
                continue

            sub.unregister()


def red_object_callback(data):
    global distance_to_red_object_flag, distance, here

    if not here:
        return

    here = False
    analyze_image(data)
    distance_to_red_object_flag = False


def move_forward_callback(data):
    global drive_flag, current_center_dist, OBSTACLE_DISTANCE_LIMIT, start_travel_center_dist
    center = data.ranges[len(data.ranges) / 2]  # Maybe scan some k items and see if one or more of them is too close?

    if start_travel_center_dist < 0:  # Init robot distance from nearest object
        start_travel_center_dist = center

    rospy.loginfo(center)
    msg = Twist()
    diff = abs(center - start_travel_center_dist)

    if center > OBSTACLE_DISTANCE_LIMIT and diff < 0.5 and drive_flag:
        msg.linear.x = 0.5 - diff
    else:
        msg.linear.x = 0.0
        drive_flag = False
    velocity_pub.publish(msg)
    time.sleep(0.001)


def turn_callback(data, angle):
    global stable_flag
    msg = Twist()

    # Converting from angles to radians
    angular_speed = 50 * 2 * math.pi / 360
    relative_angle = angle * 2 * math.pi / 360

    msg.angular.z = angular_speed

    if not stable_flag:
        # Setting the current time for distance calculus
        current_angle = 0
        t0 = rospy.Time.now().to_sec()

        while current_angle < relative_angle:
            velocity_pub.publish(msg)

            current_angle = angular_speed * (rospy.Time.now().to_sec() - t0)
            # msg.angular.z = msg.angular.z*0.9
            # t1 = rospy.Time.now().to_sec()

        msg.angular.z = 0
        velocity_pub.publish(msg)
        stable_flag = True


def move_forward():
    global start_travel_center_dist, drive_flag
    start_travel_center_dist = -1
    drive_flag = True
    sub = rospy.Subscriber("/scan", LaserScan, move_forward_callback)

    while drive_flag:
        continue

    sub.unregister()


def turn(angle):
    global start_travel_center_dist, stable_flag
    start_travel_center_dist = current_center_dist
    stable_flag = False
    sub = rospy.Subscriber("/scan", LaserScan, turn_callback, angle)

    while not stable_flag:
        continue

    sub.unregister()


def distance_to_red_object():
    global distance_to_red_object_flag, distance, here, x, y
    x = y = -1
    here = True
    distance_to_red_object_flag = True
    distance = float('inf')

    sub = rospy.Subscriber("/torso_camera/rgb/image_raw", Image, red_object_callback)

    while distance_to_red_object_flag:
        pass

    sub.unregister()

    if distance < float('inf'):
        return distance
    else:
        return -1


def find_red_object():
    degrees = 60
    counter = 1
    while distance_to_red_object() == -1 and degrees*counter < 360+40:
        turn(degrees)
        counter = counter+1
    if distance_to_red_object() < float('inf'):
        return distance_to_red_object()


def ui():
    user_input_str = ''

    while user_input_str != "q":
        user_input_str = raw_input(
            "enter command:\n1. move forward\n2. turn\n3. distance to red object\n4. find red object\nq. quit\n")

        if user_input_str == "1":
            move_forward()
        elif user_input_str == "2":
            angle = float(raw_input('enter rotation angle'))
            turn(angle)
        elif user_input_str == "3":
            dist = distance_to_red_object()
            if dist == -1:
                print 'NULL'
            else:
                print dist
        elif user_input_str == "4":
            print find_red_object()




ui()

'''
def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    hello_str = JointState()
    hello_str.position = [3, 0.5418, -1.7297, -3.1017]
    hello_str.name = ['RR', 'RL', 'FR', 'FL']
    while not rospy.is_shutdown():
        hello_str.header.stamp = rospy.Time.now()
        pub.publish(hello_str)
        rate.sleep()

talker()


sub = rospy.Subscriber("joint_states", JointState, joint_callback)
rospy.spin()
'''

''' 
if _name_ == '_main_':
    try:
        userInterface()
    except rospy.ROSInterruptException:
        pass
'''
