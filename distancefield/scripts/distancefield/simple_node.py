#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin

from distancefield.msg import Path, PathEq
import distancefield_class


class simple_node(object):
    """
    ROS node to control a simple integrator robot
    """


    def __init__(self):

        self.freq = 50.0  # Frequency of field computation in Hz

        self.pos = [0, 0, 0]  # Robot position and orientation

        self.reverse_direction = False

        # names and type of topics
        self.pose_topic_name = None
        self.pose_topic_type = None
        self.cmd_vel_topic_name = None
        self.obstacle_point_topic_name = None

        # pObstacle avoidance variables
        self.flag_follow_obstacle = None
        self.epsilon = None
        self.switch_dist_0 = None
        self.switch_dist = None
        self.obstacle_point_topic_name = None
        self.closest = [1000,1000,1000]

        # obtain the parameters
        self.vr = 0.0
        self.kf = 0.0
        self.reverse_direction = False

        # publishers
        self.pub_cmd_vel = None
        # self.pub_rviz_ref = None
        # self.pub_rviz_curve = None


        self.init_node()

        # distance field controller
        # if(self.flag_follow_obstacle):
        self.vec_field_obj = distancefield_class.distancefield_class(self.vr, self.kf, self.reverse_direction, self.flag_follow_obstacle, self.epsilon, self.switch_dist_0, self.switch_dist)
        # else:
            # self.vec_field_obj = distancefield_class.distancefield_class(self.vr, self.kf, self.reverse_direction, 0, 0)


    def run(self):
        """Execute the controller loop
        """

        rate = rospy.Rate(self.freq)

        vel_msg = Twist()


        while not rospy.is_shutdown():


            self.vec_field_obj.set_pos(self.pos)
            if(self.flag_follow_obstacle):
                self.vec_field_obj.set_closest(self.closest)

            if self.vec_field_obj.is_ready():


                [Vx,Vy,Vz,terminated] = self.vec_field_obj.vec_field_path()
                # print("Vx: %f\tVy: %f" % (Vx, Vy))


                vel_msg.linear.x = Vx
                vel_msg.linear.y = Vy
                vel_msg.linear.z = Vz
                self.pub_cmd_vel.publish(vel_msg)


            else:
                rospy.loginfo_once("\33[93mWaiting path message\33[0m")


            # self.pub_cmd_vel.publish(vel)
            # rviz_helper.send_marker_to_rviz(Vx_ref, Vy_ref, self.pos, self.pub_rviz_ref)

            rate.sleep()



    def init_node(self):
        """Initialize ROS related variables, parameters and callbacks
        :return:
        """
        rospy.init_node("simple_node")


        # parameters (description in yaml file)
        self.vr = float(rospy.get_param("~vector_field/vr", 1.0))
        self.kf = float(rospy.get_param("~vector_field/kf", 5.0))
        self.reverse_direction = rospy.get_param("~vector_field/reverse_direction", False)

        self.pose_topic_name = rospy.get_param("~topics/pose_topic_name", "tf")
        self.pose_topic_type = rospy.get_param("~topics/pose_topic_type", "TFMessage")
        self.cmd_vel_topic_name = rospy.get_param("~topics/cmd_vel_topic_name", "cmd_vel")
        self.path_topic_name = rospy.get_param("~topics/path_topic_name", "example_path")
        self.path_equation_topic_name = rospy.get_param("~topics/path_equation_topic_name", "example_path_equation")

        self.flag_follow_obstacle = rospy.get_param("~obstacle_avoidance/flag_follow_obstacle", False)
        self.epsilon = rospy.get_param("~obstacle_avoidance/epsilon", 0.5)
        self.switch_dist_0 = rospy.get_param("~obstacle_avoidance/switch_dist_0", 1.0)
        self.switch_dist = rospy.get_param("~obstacle_avoidance/switch_dist", 1.0)
        self.obstacle_point_topic_name = rospy.get_param("~obstacle_avoidance/obstacle_point_topic_name", "/closest_obstacle_point")

        # publishers
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic_name, Twist, queue_size=1)
        # self.pub_rviz_ref = rospy.Publisher("/visualization_ref_vel", Marker, queue_size=1)
        # self.pub_rviz_curve = rospy.Publisher("/visualization_path", MarkerArray, queue_size=1)

        # # subscribers
        rospy.Subscriber(self.path_topic_name, Path, self.callback_path)
        rospy.Subscriber(self.path_equation_topic_name, PathEq, self.callback_path_equation)

        if(self.flag_follow_obstacle):
            rospy.Subscriber(self.obstacle_point_topic_name, Point, self.callback_closest)

        if self.pose_topic_type == "TFMessage":
            rospy.Subscriber(self.pose_topic_name, TFMessage, self.tf_cb)
        elif self.pose_topic_type == "Pose":
            rospy.Subscriber(self.pose_topic_name, Pose, self.pose_cb)
        elif self.pose_topic_type == "Odometry":
            rospy.Subscriber(self.pose_topic_name, Odometry, self.odometry_cb)
        else:
            raise AssertionError("Invalid value for pose_topic_type:%s".format(self.pose_topic_type))


        rospy.loginfo("Vector field control configured:")
        rospy.loginfo("vr: %s, kf: %s", self.vr, self.kf)
        rospy.loginfo("reverse_direction:%s", self.reverse_direction)
        rospy.loginfo("pose_topic_name:%s, pose_topic_type:%s, cmd_vel_topic_name:%s",
                      self.pose_topic_name, self.pose_topic_type, self.cmd_vel_topic_name)
        rospy.loginfo("flag_follow_obstacle:%s",
                      self.flag_follow_obstacle)
        rospy.loginfo("obstacle_point_topic_name:%s", self.obstacle_point_topic_name)
        rospy.loginfo("flag_follow_obstacle:%s, epsilon:%s, switch_dist:%s",
                      self.flag_follow_obstacle, self.epsilon, self.switch_dist)


    def callback_closest(self,data):
        self.closest = [data.x, data.y, data.z]

    def callback_path(self, data):
        """Callback to obtain the path to be followed by the robot
        :param data: path ROS message
        """

        path_points = []
        for k in range(len(data.path.points)):
            p = data.path.points[k]
            path_points.append((p.x, p.y, p.z))

        rospy.loginfo("New path received (%d points) is closed?:%s", len(path_points), data.closed_path_flag)

        self.vec_field_obj.set_path(path_points, data.insert_n_points, data.filter_path_n_average,data.closed_path_flag)



    def callback_path_equation(self, data):
        """Callback to obtain the path to be followed by the robot
        :param data: path ROS message
        """

        rospy.loginfo("New path received (equation) is closed?:%s", data.closed_path_flag)

        self.vec_field_obj.set_equation(data.equation, data.u_i, data.u_f, data.closed_path_flag, 200)


    def tf_cb(self, data, frame_id="os1_imu_odom"):
        """Callback function to get the pose of the robot via a TF message
        :param frame_id: frame id to publish the marker
        :param data: tf ROS message
        """
        for T in data.transforms:
            if T.child_frame_id == frame_id:
                pos = (T.transform.translation.x, T.transform.translation.y, T.transform.translation.z)

                self.pos = pos

                break

    def pose_cb(self, data):
        """Callback to get the pose of the robot
        :param data: pose ROS message
        """
        pos = (data.position.x, data.position.y, data.position.z)

        self.pos = pos


    def odometry_cb(self, data):
        """Callback to get the pose from odometry data
        :param data: odometry ROS message
        """
        pos = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)

        self.pos = pos


if __name__ == '__main__':

    node = simple_node()
    node.run()