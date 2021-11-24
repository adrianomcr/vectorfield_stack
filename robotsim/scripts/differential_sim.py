#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin, sqrt, pi
import numpy as np
# import scipy as sp
# import scipy.spatial



class differential_node(object):
    """
    Navigation control using Action Server
    """


    def __init__(self):

        self.freq = 50.0  # Frequency to simulate the simple differential robot

        self.vel = [0.0, 0.0] #vx and wz

        self.state =  [0.0, 0.0, 0.0] #x, y, psi
        self.robot_radius =  0.1

        self.obtscles_pos = []
        self.obtscles_r = []


        # publishers
        self.pub_pose = None
        self.pub_closest_world = None
        self.pub_closest_body = None
        self.pub_rviz_robot = None
        self.pub_rviz_closest = None
        self.pub_rviz_obst = None
        self.pub_rviz_hist = None


        self.init_node()





    def run(self):
        """Execute the controller loop
        """
        rate = rospy.Rate(self.freq)


        pose_msg = Pose()
        point_msg = Point()
        point_msg2 = Point()


        count1 = 0

        while not rospy.is_shutdown():

            self.state[0] = self.state[0] + self.vel[0]*cos(self.state[2])*(1.0/self.freq)
            self.state[1] = self.state[1] + self.vel[0]*sin(self.state[2])*(1.0/self.freq)
            self.state[2] = self.state[2] + self.vel[1]*(1.0/self.freq)


            #Publis robots pose
            pose_msg.position.x = self.state[0]
            pose_msg.position.y = self.state[1]
            pose_msg.orientation.z = sin(self.state[2]/2.0)
            pose_msg.orientation.w = cos(self.state[2]/2.0)
            
            self.pub_pose.publish(pose_msg)


            #Compute closest point - with respect to the world frame
            n_obst = min([len(self.obtscles_r), len(self.obtscles_pos)])
            D_close = float("inf")
            o_close = 0
            for o in range(n_obst):
                Dvec = [self.state[0]-self.obtscles_pos[o][0], self.state[1]-self.obtscles_pos[o][1]]
                D = sqrt(Dvec[0]**2 + Dvec[1]**2) - self.obtscles_r[o]
                if (D<D_close):
                    o_close = o
                    D_close = D

            # Publish vector
            D_vec_close = [self.obtscles_pos[o_close][0]-self.state[0], self.obtscles_pos[o_close][1]-self.state[1]]
            D = sqrt(D_vec_close[0]**2 + D_vec_close[1]**2)
            D_hat = [D_vec_close[0]/(D+1e-8), D_vec_close[1]/(D+1e-8)]
            D = D - self.obtscles_r[o_close]
            # D_vec_close = [D_hat[0]*D, D_hat[1]*D, D_hat[2]*D]
            if D>0:
                D_vec_close = [D_vec_close[0]-D_hat[0]*self.obtscles_r[o_close], D_vec_close[1]-D_hat[1]*self.obtscles_r[o_close]]
                close_point_world = [self.state[0] + D_hat[0]*D, self.state[1] + D_hat[1]*D]
            else:
                D_vec_close = [0.0,0.0]
                close_point_world = [self.state[0], self.state[1]]

            # # Publish vector
            # point_msg.x = D_vec_close[0]
            # point_msg.y = D_vec_close[1]
            # point_msg.z = 0.0
            # Publish point
            point_msg.x = close_point_world[0]
            point_msg.y = close_point_world[1]
            point_msg.z = self.robot_height/2.0
            self.pub_closest_world.publish(point_msg)




            #Compute closest point - with respect to the robots frame

            p_cw = [[close_point_world[0]],[close_point_world[1]],[1]]
            H_bw = np.matrix([[cos(self.state[2]),-sin(self.state[2]), self.state[0]],[sin(self.state[2]),cos(self.state[2]), self.state[1]],[0, 0, 1]])
            p_cb = H_bw**(-1)*p_cw

            point_msg2.x = p_cb[0,0]
            point_msg2.y = p_cb[1,0]
            point_msg2.z = self.robot_height/2.0
            self.pub_closest_body.publish(point_msg2)
            # print ("\33[95mp_cb = [%f, %f]\33[0m" % (point_msg2.x, point_msg2.y))



            #########################################

            robot_marker_array = MarkerArray()

            #Publish robot marker to rviz
            marker_robot = Marker()
            marker_robot.header.frame_id = "world"
            marker_robot.header.stamp = rospy.Time.now()
            marker_robot.id = 0
            marker_robot.type = marker_robot.CYLINDER
            marker_robot.action = marker_robot.ADD
            marker_robot.lifetime = rospy.Duration(3)
            # Size of robot
            marker_robot.scale.x = 2*self.robot_radius
            marker_robot.scale.y = 2*self.robot_radius
            marker_robot.scale.z = self.robot_height
            #Color of the marker
            marker_robot.color.a = 0.5
            marker_robot.color.r = 0.0
            marker_robot.color.g = 0.0
            marker_robot.color.b = 0.0
            # Position of the marker
            marker_robot.pose.position.x = self.state[0]
            marker_robot.pose.position.y = self.state[1]
            marker_robot.pose.position.z = self.robot_height/2.0
            # Orientation of the marker
            marker_robot.pose.orientation.x = 0.0
            marker_robot.pose.orientation.y = 0.0
            marker_robot.pose.orientation.z = sin(self.state[2]/2.0)
            marker_robot.pose.orientation.w = cos(self.state[2]/2.0)
            # Append marker to array
            robot_marker_array.markers.append(marker_robot)


            #Publish robot marker to rviz
            marker_w1 = Marker()
            marker_w1.header.frame_id = "world"
            marker_w1.header.stamp = rospy.Time.now()
            marker_w1.id = 1
            marker_w1.type = marker_w1.CYLINDER
            marker_w1.action = marker_w1.ADD
            marker_w1.lifetime = rospy.Duration(3)
            # Size of robot
            marker_w1.scale.x = self.robot_height
            marker_w1.scale.y = self.robot_height
            marker_w1.scale.z = self.robot_height/2.0
            #Color of the marker
            marker_w1.color.a = 0.99
            marker_w1.color.r = 0.0
            marker_w1.color.g = 0.0
            marker_w1.color.b = 0.0
            # Position of the marker
            marker_w1.pose.position.x = self.state[0] + self.robot_radius*cos(self.state[2]-pi/2.0)
            marker_w1.pose.position.y = self.state[1] + self.robot_radius*sin(self.state[2]-pi/2.0)
            marker_w1.pose.position.z = self.robot_height/2.0
            # Orientation of the marker
            # Rw1 = sp.spatial.transform.Rotation.from_matrix([[cos(self.state[2]), 0, sin(self.state[2])], [sin(self.state[2]), 0, -cos(self.state[2])], [0, 1, 0]])
            # Rw1 = sp.spatial.transform.Rotation.from_matrix([[cos(psi), 0, sin(psi)], [sin(psi), 0, -cos(psi)], [0, 1, 0]])
            # qw1 = Rw1.as_quat()
            marker_w1.pose.orientation.x = cos(self.state[2]/2.0)*sin(pi/4)
            marker_w1.pose.orientation.y = sin(self.state[2]/2.0)*sin(pi/4)
            marker_w1.pose.orientation.z = sin(self.state[2]/2.0)*cos(pi/4)
            marker_w1.pose.orientation.w = cos(self.state[2]/2.0)*cos(pi/4)
            # Append marker to array
            robot_marker_array.markers.append(marker_w1)

            #Publish robot marker to rviz
            marker_w2 = Marker()
            marker_w2.header.frame_id = "world"
            marker_w2.header.stamp = rospy.Time.now()
            marker_w2.id = 2
            marker_w2.type = marker_w2.CYLINDER
            marker_w2.action = marker_w2.ADD
            marker_w2.lifetime = rospy.Duration(3)
            # Size of robot
            marker_w2.scale.x = self.robot_height
            marker_w2.scale.y = self.robot_height
            marker_w2.scale.z = self.robot_height/2.0
            #Color of the marker
            marker_w2.color.a = 0.99
            marker_w2.color.r = 0.0
            marker_w2.color.g = 0.0
            marker_w2.color.b = 0.0
            # Position of the marker
            marker_w2.pose.position.x = self.state[0] + self.robot_radius*cos(self.state[2]+pi/2.0)
            marker_w2.pose.position.y = self.state[1] + self.robot_radius*sin(self.state[2]+pi/2.0)
            marker_w2.pose.position.z = self.robot_height/2.0
            # Orientation of the marker
            # Rw1 = sp.spatial.transform.Rotation.from_matrix([[cos(self.state[2]), 0, sin(self.state[2])], [sin(self.state[2]), 0, -cos(self.state[2])], [0, 1, 0]])
            # Rw1 = sp.spatial.transform.Rotation.from_matrix([[cos(psi), 0, sin(psi)], [sin(psi), 0, -cos(psi)], [0, 1, 0]])
            # qw1 = Rw1.as_quat()
            marker_w2.pose.orientation.x = cos(self.state[2]/2.0)*sin(pi/4)
            marker_w2.pose.orientation.y = sin(self.state[2]/2.0)*sin(pi/4)
            marker_w2.pose.orientation.z = sin(self.state[2]/2.0)*cos(pi/4)
            marker_w2.pose.orientation.w = cos(self.state[2]/2.0)*cos(pi/4)
            # Append marker to array
            robot_marker_array.markers.append(marker_w2)


            #Publish robot marker to rviz
            marker_arrow = Marker()
            marker_arrow.header.frame_id = "world"
            marker_arrow.header.stamp = rospy.Time.now()
            marker_arrow.id = 3
            marker_arrow.type = marker_arrow.ARROW
            marker_arrow.action = marker_arrow.ADD
            marker_arrow.lifetime = rospy.Duration(3)
            # Size of robot
            marker_arrow.scale.x = self.robot_radius*1.5
            marker_arrow.scale.y = self.robot_radius/4.0
            marker_arrow.scale.z = self.robot_radius/4.0
            #Color of the marker
            marker_arrow.color.a = 0.99
            marker_arrow.color.r = 0.0
            marker_arrow.color.g = 0.0
            marker_arrow.color.b = 1.0
            # Position of the marker
            marker_arrow.pose.position.x = self.state[0] - (self.robot_radius*1.5/2.0)*cos(self.state[2])
            marker_arrow.pose.position.y = self.state[1] - (self.robot_radius*1.5/2.0)*sin(self.state[2])
            marker_arrow.pose.position.z = self.robot_height/2.0
            # Orientation of the marker
            # Rw1 = sp.spatial.transform.Rotation.from_matrix([[cos(self.state[2]), 0, sin(self.state[2])], [sin(self.state[2]), 0, -cos(self.state[2])], [0, 1, 0]])
            # Rw1 = sp.spatial.transform.Rotation.from_matrix([[cos(psi), 0, sin(psi)], [sin(psi), 0, -cos(psi)], [0, 1, 0]])
            # qw1 = Rw1.as_quat()
            marker_arrow.pose.orientation.x = 0
            marker_arrow.pose.orientation.y = 0
            marker_arrow.pose.orientation.z = sin(self.state[2]/2.0)
            marker_arrow.pose.orientation.w = cos(self.state[2]/2.0)
            # Append marker to array
            robot_marker_array.markers.append(marker_arrow)


            # Publish marker
            self.pub_rviz_robot.publish(robot_marker_array)



            #########################################



            #Publish closest point to rviz
            marker_closest = Marker()

            marker_closest.header.frame_id = "world"
            marker_closest.header.stamp = rospy.Time.now()
            marker_closest.id = 0
            marker_closest.type = marker_closest.SPHERE
            marker_closest.action = marker_closest.ADD
            marker_closest.lifetime = rospy.Duration(3)
            #Size of sphere
            marker_closest.scale.x = self.robot_radius
            marker_closest.scale.y = self.robot_radius
            marker_closest.scale.z = self.robot_radius
            #Color of the marker
            marker_closest.color.a = 1.0
            marker_closest.color.r = 1.0
            marker_closest.color.g = 1.0
            marker_closest.color.b = 1.0
            # Position of the marker
            marker_closest.pose.position.x = close_point_world[0]
            marker_closest.pose.position.y = close_point_world[1]
            marker_closest.pose.position.z = 0.0
            # Orientation of the marker
            marker_closest.pose.orientation.x = 0.0
            marker_closest.pose.orientation.y = 0.0
            marker_closest.pose.orientation.z = 0.0
            marker_closest.pose.orientation.w = 1.0

            # Publish marker
            self.pub_rviz_closest.publish(marker_closest)


            #Publish obstacles markers to rviz
            count1 = count1 + 1
            if (count1 > self.freq):
                count1 = 0

                points_marker = MarkerArray()
                for i in range(min([len(self.obtscles_r), len(self.obtscles_pos)])):
                    marker = Marker()
                    marker.header.frame_id = "world"
                    marker.header.stamp = rospy.Time.now()
                    marker.id = i
                    marker.type = marker.CYLINDER
                    marker.action = marker.ADD
                    marker.lifetime = rospy.Duration(3)
                    # Size of cylinder
                    marker.scale.x = 2*self.obtscles_r[i]
                    marker.scale.y = 2*self.obtscles_r[i]
                    marker.scale.z = self.robot_height*2
                    # Color and transparency
                    marker.color.a = 0.7
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    # Pose
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0

                    marker.pose.position.x = self.obtscles_pos[i][0]
                    marker.pose.position.y = self.obtscles_pos[i][1]
                    marker.pose.position.z = self.robot_height

                    # Append marker to array
                    points_marker.markers.append(marker)

                # Publish marker array
                self.pub_rviz_obst.publish(points_marker)


            #Publish robots history to rviz
            delta = 0.05
            approx_len = 3.0
            if(sqrt((self.state[0]-self.history[-1][0])**2+(self.state[1]-self.history[-1][1])**2) > delta):
                self.history.append([self.state[0],self.state[1]])
                # print(len(self.history))
                # print("meleca")
                if(len(self.history)*delta > approx_len):
                    self.history.pop(0)


            points_marker = MarkerArray()
            for i in range(len(self.history)):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.lifetime = rospy.Duration(3)
                # Size of sphere
                marker.scale.x = 2*self.robot_radius/3.0
                marker.scale.y = 2*self.robot_radius/3.0
                marker.scale.z = 2*self.robot_radius/3.0
                # Color and transparency
                marker.color.a = 0.5
                marker.color.r = 0.7
                marker.color.g = 0.7
                marker.color.b = 0.7
                # Pose
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.pose.position.x = self.history[i][0]
                marker.pose.position.y = self.history[i][1]
                marker.pose.position.z = self.robot_height/2.0

                # Append marker to array
                points_marker.markers.append(marker)

            # Publish marker array
            self.pub_rviz_hist.publish(points_marker)





            rate.sleep()



    def init_node(self):
        """Initialize ROS related variables, parameters and callbacks
        :return:
        """
        rospy.init_node("differential_sim")



        # parameters (description in yaml file)
        self.state = rospy.get_param("~state_0", 1.0)
        self.robot_radius = float(rospy.get_param("~robot_radius", 1.0))
        self.robot_height = float(rospy.get_param("~robot_height", 1.0))
        self.obtscles_pos = rospy.get_param("~obtscles_pos", [])
        self.obtscles_r = rospy.get_param("~obtscles_r", [])
        self.history = []
        self.history.append([self.state[0], self.state[1], self.state[2]])

        print("self.state: ", self.state)
        print("self.robot_radius: ", self.robot_radius)
        print("self.robot_height: ", self.robot_height)
        print("self.obtscles_pos: ", self.obtscles_pos)
        print("self.obtscles_r: ", self.obtscles_r)

        # publishers
        self.pub_pose = rospy.Publisher("/differential/pose", Pose, queue_size=1)
        self.pub_closest_world = rospy.Publisher("/differential/closest_point_world", Point, queue_size=1)
        self.pub_closest_body = rospy.Publisher("/differential/closest_point_body", Point, queue_size=1)
        self.pub_rviz_robot = rospy.Publisher("/differential/robot", MarkerArray, queue_size=1)
        self.pub_rviz_closest = rospy.Publisher("/differential/closest_marker", Marker, queue_size=1)
        self.pub_rviz_obst = rospy.Publisher("/differential/obstacles", MarkerArray, queue_size=1)
        self.pub_rviz_hist = rospy.Publisher("/differential/history", MarkerArray, queue_size=1)


        # self.pub_rviz_robot_w1 = rospy.Publisher("/differential/robot_w1", Marker, queue_size=1)

        # subscribers
        rospy.Subscriber("/differential/cmd_vel", Twist, self.callback_vel)




    def callback_vel(self, data):
        """Callback to get the reference velocity for the robot
        :param data: pose ROS message
        """
        vel = [data.linear.x, data.angular.z]

        self.vel = vel




if __name__ == '__main__':
    node = differential_node()
    node.run()