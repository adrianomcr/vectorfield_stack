#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin, sqrt, atan2
import numpy as np

import time

from distancefield.msg import Path, PathEq
import quadrobot_class
import math_utils.math_utils as MU

class quad_node(object):
    """
    ROS node to control a quadcopter on the AcroRate mode
    """


    def __init__(self):

        self.freq = 70.0  # Frequency of field computation in Hz

        self.state = [0,0,0, 1,0,0,0, 0,0,0]  # Robot position and orientation

        self.reverse_direction = False

        # names and type of topics
        self.pose_topic_name = None
        self.pose_topic_type = None
        self.acrorate_cmd_topic_name = None
        self.obstacle_point_topic_name = None

        # Obstacle avoidance variables
        self.flag_follow_obstacle = None
        self.epsilon = None
        self.switch_dist_0 = None
        self.switch_dist = None
        self.closest = [1000,1000,1000]
        self.closest_world = [1000,1000,1000]
        

        # obtain the parameters
        self.vr = 0.0
        self.kf = 0.0
        self.reverse_direction = False
        self.m = 0.0
        self.kv = 0.0
        self.kf = 0.0

        # publishers
        # self.pub_cmd_vel = None
        self.pub_acrorate = None
        # self.pub_rviz_ref = None
        # self.pub_rviz_curve = None


        self.init_node()

        # distance field controller
        self.quad_robot_obj = quadrobot_class.quadrobot_class(self.vr, self.kf, self.reverse_direction, self.flag_follow_obstacle, self.epsilon, self.switch_dist_0, self.switch_dist, self.m, self.kv, self.kw)





    def run(self):
        """Execute the controller loop
        """
        rate = rospy.Rate(self.freq)

        # vel_msg = Twist()
        wheels_msg = Float32MultiArray()
        acrorate_msg = Quaternion()

        time.sleep(2)

        while not rospy.is_shutdown():


            self.quad_robot_obj.set_state(self.state)
            

            if self.quad_robot_obj.vec_field_obj.is_ready():

                if(self.flag_follow_obstacle):
                    self.quad_robot_obj.vec_field_obj.set_closest(self.closest_world)

                self.quad_robot_obj.control_step()
                [tau, omega] = self.quad_robot_obj.get_acrorate()

                acrorate_msg.w = tau
                acrorate_msg.x = omega[0]
                acrorate_msg.y = omega[1]
                acrorate_msg.z = omega[2]
                self.pub_acrorate.publish(acrorate_msg)

            else:
                rospy.loginfo_once("\33[93mWaiting path message\33[0m")


            # self.pub_cmd_vel.publish(vel)
            # rviz_helper.send_marker_to_rviz(Vx_ref, Vy_ref, self.pos, self.pub_rviz_ref)

            rate.sleep()



    def init_node(self):
        """Initialize ROS related variables, parameters and callbacks
        :return:
        """
        rospy.init_node("quad_node")


        # parameters (description in yaml file)
        self.vr = float(rospy.get_param("~vector_field/vr", 1.0))
        self.kf = float(rospy.get_param("~vector_field/kf", 5.0))
        self.reverse_direction = rospy.get_param("~vector_field/reverse_direction", False)
        self.m = rospy.get_param("~quadrobot/m", 1.0)
        self.kv = rospy.get_param("~quadrobot/kv", 1.0)
        self.kw = rospy.get_param("~quadrobot/kw", 1.0)

        self.pose_topic_name = rospy.get_param("~topics/pose_topic_name", "tf")
        self.pose_topic_type = rospy.get_param("~topics/pose_topic_type", "Odometry")
        self.acrorate_cmd_topic_name = rospy.get_param("~topics/acrorate_cmd_topic_name", "wheels_speeds")
        self.path_topic_name = rospy.get_param("~topics/path_topic_name", "example_path")
        self.path_equation_topic_name = rospy.get_param("~topics/path_equation_topic_name", "example_path_equation")

        self.flag_follow_obstacle = rospy.get_param("~obstacle_avoidance/flag_follow_obstacle", False)
        self.epsilon = rospy.get_param("~obstacle_avoidance/epsilon", 0.5)
        self.switch_dist_0 = rospy.get_param("~obstacle_avoidance/switch_dist_0", 1.5)
        self.switch_dist = rospy.get_param("~obstacle_avoidance/switch_dist", 1.0)
        self.obstacle_point_topic_name = rospy.get_param("~obstacle_avoidance/obstacle_point_topic_name", "/closest_obstacle_point")

        # publishers
        # self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic_name, Twist, queue_size=1)
        self.pub_acrorate = rospy.Publisher(self.acrorate_cmd_topic_name, Quaternion, queue_size=1)
        # self.pub_rviz_ref = rospy.Publisher("/visualization_ref_vel", Marker, queue_size=1)
        # self.pub_rviz_curve = rospy.Publisher("/visualization_path", MarkerArray, queue_size=1)

        # # subscribers
        rospy.Subscriber(self.path_topic_name, Path, self.callback_path)
        rospy.Subscriber(self.path_equation_topic_name, PathEq, self.callback_path_equation)

        if(self.flag_follow_obstacle):
            # rospy.Subscriber(self.obstacle_point_topic_name, Point, self.callback_closest_body)
            rospy.Subscriber(self.obstacle_point_topic_name, Point, self.callback_closest)
        # rospy.Subscriber(self.obstacle_point_topic_name, Point, self.obstacle_point_cb)


        if self.pose_topic_type == "Odometry":
            rospy.Subscriber(self.pose_topic_name, Odometry, self.odometry_cb)
        else:
            raise AssertionError("Invalid value for pose_topic_type:%s".format(self.pose_topic_type))


        rospy.loginfo("Vector field control configured:")
        rospy.loginfo("vr: %f, kf: %f", self.vr, self.kf)
        rospy.loginfo("m: %f, kv: %f, kf: %f", self.m, self.kv, self.kv)
        rospy.loginfo("reverse_direction:%s", self.reverse_direction)
        rospy.loginfo("pose_topic_name:%s, pose_topic_type:%s, acrorate_cmd_topic_name:%s",
                      self.pose_topic_name, self.pose_topic_type, self.acrorate_cmd_topic_name)
        rospy.loginfo("flag_follow_obstacle:%s",
                      self.flag_follow_obstacle)
        rospy.loginfo("obstacle_point_topic_name:%s", self.obstacle_point_topic_name)
        rospy.loginfo("flag_follow_obstacle:%s, epsilon:%s, switch_dist:%s",
                      self.flag_follow_obstacle, self.epsilon, self.switch_dist)

    def callback_closest_body(self,data):
        self.closest = [data.x, data.y, data.z]

        # CORRECT THE TRANSFORMATION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # COMPUTE WORLD POINT FROM THE BODY POINT
        r = sqrt(data.x*data.x + data.y*data.y)
        theta = atan2(data.y,data.x)

        self.closest_world = [self.state[0]+r*cos(self.state[2]+theta), self.state[1]+r*sin(self.state[2]+theta), 0.0+data.z]


    def callback_closest(self,data):
        self.closest = [data.x, data.y, data.z]


        self.closest_world = [self.closest[0], self.closest[1], self.closest[2]]



    def callback_path(self, data):
        """Callback to obtain the path to be followed by the robot
        :param data: path ROS message
        """

        path_points = []
        for k in range(len(data.path.points)):
            p = data.path.points[k]
            path_points.append((p.x, p.y, p.z))

        rospy.loginfo("New path received (%d points) is closed?:%s", len(path_points), data.closed_path_flag)

        self.quad_robot_obj.vec_field_obj.set_path(path_points, data.insert_n_points, data.filter_path_n_average,data.closed_path_flag)



    def callback_path_equation(self, data):
        """Callback to obtain the path to be followed by the robot
        :param data: path ROS message
        """

        rospy.loginfo("New path received (equation) is closed?:%s", data.closed_path_flag)

        self.quad_robot_obj.vec_field_obj.set_equation(data.equation, data.u_i, data.u_f, data.closed_path_flag, 200)



    def odometry_cb(self, data):
        """Callback to get the pose from odometry data
        :param data: odometry ROS message
        """
        pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]

        quat = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]

        vel_b = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]

        R_bw = MU.quat2rotm(quat)
        vel_w = np.matrix(R_bw)*np.matrix(vel_b).transpose()
        vel_w = vel_w.transpose().tolist()[0]

        self.state[0] = pos[0]
        self.state[1] = pos[1]
        self.state[2] = pos[2]

        self.state[3] = quat[0]
        self.state[4] = quat[1]
        self.state[5] = quat[2]
        self.state[6] = quat[3]

        self.state[7] = vel_w[0]
        self.state[8] = vel_w[1]
        self.state[9] = vel_w[2]
        


if __name__ == '__main__':

    node = quad_node()
    node.run()