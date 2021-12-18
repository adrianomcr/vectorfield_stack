#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
# from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin, sqrt

# import rviz_helper
# import vec_field_controller
# from distancefield.msg import Path
# import distancefield_class



class integrator_node(object):
    """
    Navigation control using Action Server
    """


    def __init__(self):

        self.freq = 50.0  # Frequency to simulate the simple integrator robot

        self.vel = [0.0, 0.0, 0.0]

        self.pos =  [0.0, 0.0, 0.0]
        self.robot_radius =  0.1

        self.obtscles_pos = []
        self.obtscles_r = []


        # publishers
        self.pub_pose = None
        self.pub_rviz_robot = None
        self.pub_rviz_obst = None



        self.init_node()





    def run(self):
        """Execute the controller loop
        """
        rate = rospy.Rate(self.freq)


        pose_msg = Pose()
        point_msg = Point()


        count1 = 0
        count2 = 0

        while not rospy.is_shutdown():

            self.pos[0] = self.pos[0] + self.vel[0]*(1.0/self.freq)
            self.pos[1] = self.pos[1] + self.vel[1]*(1.0/self.freq)
            self.pos[2] = self.pos[2] + self.vel[2]*(1.0/self.freq)


            #Publis robots pose
            pose_msg.position.x = self.pos[0]
            pose_msg.position.y = self.pos[1]
            pose_msg.position.z = self.pos[2]

            self.pub_pose.publish(pose_msg)



            #Compute closest point - with respect to the world frame
            n_obst = min([len(self.obtscles_r), len(self.obtscles_pos)])
            D_close = float("inf")
            o_close = 0
            for o in range(n_obst):
                Dvec = [self.pos[0]-self.obtscles_pos[o][0], self.pos[1]-self.obtscles_pos[o][1], self.pos[2]-self.obtscles_pos[o][2]]
                D = sqrt(Dvec[0]**2 + Dvec[1]**2 + Dvec[2]**2) - self.obtscles_r[o]
                if (D<D_close):
                    o_close = o
                    D_close = D

            # Publish vector
            D_vec_close = [self.obtscles_pos[o_close][0]-self.pos[0], self.obtscles_pos[o_close][1]-self.pos[1], self.obtscles_pos[o_close][2]-self.pos[2]]
            D = sqrt(D_vec_close[0]**2 + D_vec_close[1]**2 + D_vec_close[2]**2)
            D_hat = [D_vec_close[0]/(D+1e-8), D_vec_close[1]/(D+1e-8), D_vec_close[2]/(D+1e-8)]
            D = D - self.obtscles_r[o_close]
            # D_vec_close = [D_hat[0]*D, D_hat[1]*D, D_hat[2]*D]
            if D>0:
                D_vec_close = [D_vec_close[0]-D_hat[0]*self.obtscles_r[o_close], D_vec_close[1]-D_hat[1]*self.obtscles_r[o_close], D_vec_close[2]-D_hat[2]*self.obtscles_r[o_close]]
                close_point_world = [self.pos[0] + D_hat[0]*D, self.pos[1] + D_hat[1]*D, self.pos[2] + D_hat[2]*D]
            else:
                D_vec_close = [0.0,0.0,0.0]
                close_point_world = [self.pos[0], self.pos[1], self.pos[2]]

            # # Publish vector
            # point_msg.x = D_vec_close[0]
            # point_msg.y = D_vec_close[1]
            # point_msg.z = D_vec_close[2]
            # Publish point
            point_msg.x = close_point_world[0]
            point_msg.y = close_point_world[1]
            point_msg.z = close_point_world[2]
            self.pub_closest.publish(point_msg)




            #Publish robot marker to rviz
            marker_robot = Marker()

            marker_robot.header.frame_id = "world"
            marker_robot.header.stamp = rospy.Time.now()
            marker_robot.id = 0
            marker_robot.type = marker_robot.SPHERE
            marker_robot.action = marker_robot.ADD
            marker_robot.lifetime = rospy.Duration(3)
            # Size of robot
            marker_robot.scale.x = 2*self.robot_radius
            marker_robot.scale.y = 2*self.robot_radius
            marker_robot.scale.z = 2*self.robot_radius
            #Color of the marker
            marker_robot.color.a = 0.9
            marker_robot.color.r = 0.0
            marker_robot.color.g = 0.0
            marker_robot.color.b = 0.0
            # Position of the marker
            marker_robot.pose.position.x = self.pos[0]
            marker_robot.pose.position.y = self.pos[1]
            marker_robot.pose.position.z = self.pos[2]
            # Orientation of the marker
            marker_robot.pose.orientation.x = 0.0
            marker_robot.pose.orientation.y = 0.0
            marker_robot.pose.orientation.z = 0.0
            marker_robot.pose.orientation.w = 1.0

            # Publish marker
            self.pub_rviz_robot.publish(marker_robot)


            #Publish robot marker to rviz
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
            marker_closest.pose.position.z = close_point_world[2]
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
                    marker.type = marker.SPHERE
                    marker.action = marker.ADD
                    marker.lifetime = rospy.Duration(3)
                    # Size of sphere
                    marker.scale.x = 2*self.obtscles_r[i]
                    marker.scale.y = 2*self.obtscles_r[i]
                    marker.scale.z = 2*self.obtscles_r[i]
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
                    marker.pose.position.z = self.obtscles_pos[i][2]

                    # Append marker to array
                    points_marker.markers.append(marker)

                # Publish marker array
                self.pub_rviz_obst.publish(points_marker)


            #Publish robots history to rviz
            delta = 0.05
            approx_len = 3.0
            if(sqrt((self.pos[0]-self.history[-1][0])**2+(self.pos[1]-self.history[-1][1])**2+(self.pos[2]-self.history[-1][2])**2) > delta):
                self.history.append([self.pos[0],self.pos[1],self.pos[2]])
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
                marker.pose.position.z = self.history[i][2]

                # Append marker to array
                points_marker.markers.append(marker)

            # Publish marker array
            self.pub_rviz_hist.publish(points_marker)





            rate.sleep()



    def init_node(self):
        """Initialize ROS related variables, parameters and callbacks
        :return:
        """
        rospy.init_node("integrator_sim")



        # parameters (description in yaml file)
        self.pos = rospy.get_param("~pos_0", 1.0)
        self.robot_radius = float(rospy.get_param("~robot_radius", 5.0))
        self.obtscles_pos = rospy.get_param("~obtscles_pos", [])
        self.obtscles_r = rospy.get_param("~obtscles_r", [])
        self.history = []
        self.history.append([self.pos[0], self.pos[1], self.pos[2]])

        print("self.pos: ", self.pos)
        print("self.robot_radius: ", self.robot_radius)
        print("self.obtscles_pos: ", self.obtscles_pos)
        print("self.obtscles_r: ", self.obtscles_r)

        # publishers
        self.pub_pose = rospy.Publisher("/integrator/pose", Pose, queue_size=1)
        self.pub_closest = rospy.Publisher("/integrator/closest_point", Point, queue_size=1)
        self.pub_rviz_robot = rospy.Publisher("/integrator/robot", Marker, queue_size=1)
        self.pub_rviz_closest = rospy.Publisher("/integrator/closest_marker", Marker, queue_size=1)
        self.pub_rviz_obst = rospy.Publisher("/integrator/obstacles", MarkerArray, queue_size=1)
        self.pub_rviz_hist = rospy.Publisher("/integrator/history", MarkerArray, queue_size=1)

        # subscribers
        rospy.Subscriber("/integrator/vel", Twist, self.callback_vel)




    def callback_vel(self, data):
        """Callback to get the reference velocity for the robot
        :param data: pose ROS message
        """
        vel = [data.linear.x, data.linear.y, data.linear.z]

        self.vel = vel



if __name__ == '__main__':
    node = integrator_node()
    node.run()
