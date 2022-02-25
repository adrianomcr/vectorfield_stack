#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin, sqrt, atan2


# from distancefield.import_me_if_you_can import say_it_works


# import rviz_helper
# import vec_field_controller
from distancefield.msg import Path, PathEq
# import distancefield_class
# import distancefield
# from distancefield import distancefield_class
# import distancefield.distancefield_class
import groundrobot_class

import distancefield.distancefield_class


class differential_node(object):
    """
    Navigation control using Action Server
    """


    def __init__(self):

        self.freq = 50.0  # Frequency of field computation in Hz

        self.state = [0, 0, 0]  # Robot position and orientation
        # self.pos = [0, 0, 0]  # Robot position and orientation
        # self.rpy = [0, 0, 0]
        #self.rpy = [1, 0, 0, 0]

        self.reverse_direction = False

        # names and type of topics
        self.pose_topic_name = None
        self.pose_topic_type = None
        self.cmd_vel_topic_name = None
        self.obstacle_point_body_topic_name = None

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
        self.d_feedback = 0.1
        self.move_backwards = False

        # publishers
        self.pub_cmd_vel = None
        # self.pub_rviz_ref = None
        # self.pub_rviz_curve = None


        self.init_node()

        # distance field controller
        # if(self.flag_follow_obstacle):
        # self.vec_field_obj = distancefield_class.distancefield_class(self.vr, self.kf, self.reverse_direction, self.flag_follow_obstacle, self.epsilon, self.switch_dist)
        self.ground_robot_obj = groundrobot_class.groundrobot_class(self.vr, self.kf, self.reverse_direction, self.flag_follow_obstacle, self.epsilon, self.switch_dist_0, self.switch_dist, self.d_feedback, self.move_backwards)
        # else:
            # self.vec_field_obj = distancefield_class.distancefield_class(self.vr, self.kf, self.reverse_direction, 0, 0)




    def run(self):
        """Execute the controller loop
        """
        rate = rospy.Rate(self.freq)

        vel_msg = Twist()


        while not rospy.is_shutdown():

            self.ground_robot_obj.set_state(self.state)
            

            # print(self.ground_robot_obj.vec_field_obj.is_ready())
            if self.ground_robot_obj.vec_field_obj.is_ready():

                if(self.flag_follow_obstacle):
                    self.ground_robot_obj.vec_field_obj.set_closest(self.closest_world)


                # [Vx,Vy,Vz,terminated] = self.vec_field_obj.vec_field_path()
                [Vx,Wz] = self.ground_robot_obj.get_vw()
                #print("\33[96mVx: %f Wz: %f\33[0m" % (Vx, Wz))
                #print("")

                # print(self.flag_follow_obstacle)
            # if(self.flag_follow_obstacle):

            #     closest_vec = [self.closest_world[0]-self.pos[0], self.closest_world[1]-self.pos[1], self.closest_world[2]-self.pos[2]]

            #     Do = math.sqrt(closest_vec[0]**2 + closest_vec[1]**2 + closest_vec[2]**2)

            #     closest_hat = [closest_vec[0]/(Do+1e-8), closest_vec[1]/(Do+1e-8), closest_vec[2]/(Do+1e-8)]

            #     if (Do < self.D_hist):
            #         self.D_hist = Do
            #     # print (Do, self.D_hist)


            #     if(Do<self.switch_dist and (closest_vec[0]*Vx+closest_vec[1]*Vy+closest_vec[2]*Vz)>0):



                #IDENTIFY CLOSEST POINT

                #CALL FUNCTION FO CONTOUR



                vel_msg.linear.x = Vx
                vel_msg.angular.z = Wz
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
        rospy.init_node("differential_node")


        # parameters (description in yaml file)
        self.vr = float(rospy.get_param("~vector_field/vr", 1.0))
        self.kf = float(rospy.get_param("~vector_field/kf", 5.0))
        self.reverse_direction = rospy.get_param("~vector_field/reverse_direction", False)
        self.d_feedback = float(rospy.get_param("~groundrobot/d_feedback", 0.1))
        self.move_backwards = rospy.get_param("~groundrobot/move_backwards", False)

        self.pose_topic_name = rospy.get_param("~topics/pose_topic_name", "tf")
        self.pose_topic_type = rospy.get_param("~topics/pose_topic_type", "TFMessage")
        self.cmd_vel_topic_name = rospy.get_param("~topics/cmd_vel_topic_name", "cmd_vel")
        self.path_topic_name = rospy.get_param("~topics/path_topic_name", "example_path")
        self.path_equation_topic_name = rospy.get_param("~topics/path_equation_topic_name", "example_path_equation")

        self.flag_follow_obstacle = rospy.get_param("~obstacle_avoidance/flag_follow_obstacle", False)
        self.epsilon = rospy.get_param("~obstacle_avoidance/epsilon", 0.5)
        self.switch_dist_0 = rospy.get_param("~obstacle_avoidance/switch_dist_0", 1.0)
        self.switch_dist = rospy.get_param("~obstacle_avoidance/switch_dist", 1.0)
        self.obstacle_point_body_topic_name = rospy.get_param("~obstacle_avoidance/obstacle_point_body_topic_name", "/closest_obstacle_point")

        # publishers
        self.pub_cmd_vel = rospy.Publisher(self.cmd_vel_topic_name, Twist, queue_size=1)
        # self.pub_rviz_ref = rospy.Publisher("/visualization_ref_vel", Marker, queue_size=1)
        # self.pub_rviz_curve = rospy.Publisher("/visualization_path", MarkerArray, queue_size=1)

        # # subscribers
        rospy.Subscriber(self.path_topic_name, Path, self.callback_path)
        rospy.Subscriber(self.path_equation_topic_name, PathEq, self.callback_path_equation)

        if(self.flag_follow_obstacle):
            rospy.Subscriber(self.obstacle_point_body_topic_name, Point, self.callback_closest_body)


        # rospy.Subscriber(self.obstacle_point_body_topic_name, Point, self.obstacle_point_cb)

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
        rospy.loginfo("d: %s, move_backwards: %s", self.d_feedback, self.move_backwards)
        rospy.loginfo("reverse_direction:%s", self.reverse_direction)
        rospy.loginfo("pose_topic_name:%s, pose_topic_type:%s, cmd_vel_topic_name:%s",
                      self.pose_topic_name, self.pose_topic_type, self.cmd_vel_topic_name)
        rospy.loginfo("flag_follow_obstacle:%s",
                      self.flag_follow_obstacle)
        rospy.loginfo("obstacle_point_body_topic_name:%s", self.obstacle_point_body_topic_name)
        rospy.loginfo("flag_follow_obstacle:%s, epsilon:%s, switch_dist:%s",
                      self.flag_follow_obstacle, self.epsilon, self.switch_dist)



    def callback_closest_body(self,data):
        self.closest = [data.x, data.y, data.z]


        # COMPUTE WORLD POINT FROM THE BODY POINT
        r = sqrt(data.x*data.x + data.y*data.y)
        theta = atan2(data.y,data.x)

        self.closest_world = [self.state[0]+r*cos(self.state[2]+theta), self.state[1]+r*sin(self.state[2]+theta), 0.0+data.z]



    def callback_path(self, data):
        """Callback to obtain the path to be followed by the robot
        :param data: path ROS message
        """

        path_points = []
        for k in range(len(data.path.points)):
            p = data.path.points[k]
            path_points.append((p.x, p.y, p.z))

        rospy.loginfo("New path received (%d points) is closed?:%s", len(path_points), data.closed_path_flag)

        self.ground_robot_obj.vec_field_obj.set_path(path_points, data.insert_n_points, data.filter_path_n_average,data.closed_path_flag)



    def callback_path_equation(self, data):
        """Callback to obtain the path to be followed by the robot
        :param data: path ROS message
        """

        rospy.loginfo("New path received (equation) is closed?:%s", data.closed_path_flag)

        self.ground_robot_obj.vec_field_obj.set_equation(data.equation, data.u_i, data.u_f, data.closed_path_flag, 200)



    def tf_cb(self, data, frame_id="os1_imu_odom"):
        """Callback function to get the pose of the robot via a TF message
        :param frame_id: frame id to publish the marker
        :param data: tf ROS message
        """
        for T in data.transforms:
            if T.child_frame_id == frame_id:
                pos = (T.transform.translation.x, T.transform.translation.y, T.transform.translation.z)

                # x_q = T.transform.rotation.x
                # y_q = T.transform.rotation.y
                # z_q = T.transform.rotation.z
                # w_q = T.transform.rotation.w
                # rpy = euler_from_quaternion([x_q, y_q, z_q, w_q])

                # self.vec_field_obj.set_pos(pos)
                # self.vec_field_obj.set_pos(pos, rpy)
                self.pos = pos
                # self.rpy = rpy
                break



    def pose_cb(self, data):
        """Callback to get the pose of the robot
        :param data: pose ROS message
        """
        pos = (data.position.x, data.position.y, data.position.z)

        x_q = data.orientation.x
        y_q = data.orientation.y
        z_q = data.orientation.z
        w_q = data.orientation.w
        rpy = euler_from_quaternion([x_q, y_q, z_q, w_q])

        self.state = [pos[0], pos[1], rpy[2]]



    def odometry_cb(self, data):
        """Callback to get the pose from odometry data
        :param data: odometry ROS message
        """
        pos = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)

        # x_q = data.pose.pose.orientation.x
        # y_q = data.pose.pose.orientation.y
        # z_q = data.pose.pose.orientation.z
        # w_q = data.pose.pose.orientation.w
        # rpy = euler_from_quaternion([x_q, y_q, z_q, w_q])

        #Consider the position of the control point, instead of the robot's center
        # pos = (pos[0] + self.d_feedback*cos(rpy[2]), pos[1] + self.d_feedback*sin(rpy[2]), pos[2])


        # self.vec_field_obj.set_pos(pos, rpy)
        # self.vec_field_obj.set_pos(pos)
        self.pos = pos
        # self.rpy = rpy




if __name__ == '__main__':


    node = differential_node()
    node.run()