#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import numpy as np
from lab5.msg import PlotPoint
from lab5.msg import MapPoints
from lab5.msg import SimPose
import math


class Pose():
    def __init__(self):
        # Odom values
        self.x = 0
        self.y = 0
        self.a = 0

        # Ground Truth values
        self.gt_x = 0
        self.gt_y = 0
        self.gt_a = 0


class Robot():
    def __init__(self):
        print ("Using python version:", sys.version)

        self.pose = Pose()

        self.laser_range = 0.0

        self.subscriber_pose_time_delay = 0.01

        # Temp variables
        self.temp_pose = Pose()

    def odom_callback(self, data):
        self.temp_pose.x = data.odom_x
        self.temp_pose.y = data.odom_y
        self.temp_pose.a = data.odom_a
        self.temp_pose.gt_x = data.gt_x
        self.temp_pose.gt_y = data.gt_y
        self.temp_pose.gt_a = data.gt_a

        self.pose = self.temp_pose


        # time.sleep(self.subscriber_pose_time_delay)

    def laser_callback(self, data):
        self.laser_range = data.ranges[0]


    def set_vel(self, v, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w

        self.vel_pub.publish(cmd_vel)

    def get_pose(self):
        return (self.pose.x, self.pose.y, self.pose.a)

    def get_gt_pose(self):
        return (self.pose.gt_x, self.pose.gt_y, self.pose.gt_a)

    def get_laser_data(self):
        return self.laser_range

    def initialize(self):
        rospy.init_node('lab5_robot_controller', anonymous=True)
        rospy.Subscriber("/sim_pose",
                         SimPose,
                         self.odom_callback,
                         queue_size=1)

        rospy.Subscriber("/base_scan",
                         LaserScan,
                         self.laser_callback,
                         queue_size=1)

        self.vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist,
                                       queue_size=1)

        self.traj_pub = rospy.Publisher('/traj_points',
                                        PlotPoint,
                                        queue_size=1)

