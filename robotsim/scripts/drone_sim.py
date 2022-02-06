#!/usr/bin/env python
# -*- coding: utf-8 -*-



import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin, sqrt, pi
import numpy as np
#import scipy as sp
#import scipy.spatial



class drone_node(object):
    """
    Navigation control using Action Server
    """


    def __init__(self):

        self.freq = 100.0  # Frequency to simulate the simple drone robot

        # self.vel = [0.0, 0.0] #vx and wz
        self.tau = 9.81
        self.omega = [0.0, 0.0, 0.0]

        self.state =  [0.0, 0.0, 0.0] #x, y, psi
        self.robot_radius =  0.1

        self.obtscles_pos = []
        self.obtscles_r = []


        # publishers
        self.pub_pose = None
        self.pub_odom = None
        self.pub_rviz_robot = None
        self.pub_rviz_obst = None



        self.init_node()






    # Unit quaternion to rotation matrix
    def quat_derivative(self, q, w):
        # w x y z
        #velocity w in the world frame

        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]

        f = [0,0,0,0]
        f[0] = -0.5*(w[0]*qx+w[1]*qy+w[2]*qz)
        f[1] = 0.5*(w[0]*qw+w[1]*qz-w[2]*qy)
        f[2] = 0.5*(w[1]*qw+w[2]*qx-w[0]*qz)
        f[3] = 0.5*(w[2]*qw+w[0]*qy-w[1]*qx)

        return f


    # Unit quaternion to rotation matrix
    def quat2rotm(self,q):
        # w x y z

        qw = q[0]
        qx = q[1]
        qy = q[2]
        qz = q[3]

        Rot = [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
               [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
               [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]]; #this was checked on matlab
               
        return Rot


    def angle2rotm(self, rpy):

        phi = rpy[0];
        theta = rpy[1];
        psi = rpy[2];
        # Get rotation matrix
        Rot << [[(cos(theta)*cos(psi)), (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)), (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))],
                [(cos(theta)*sin(psi)), (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)), (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))],
                [(-sin(theta)), (sin(phi)*cos(theta)), (cos(phi)*cos(theta))]]

        return Rot;


    #Function to convert euler angles to a quaternion
    def eul2quat(self, ang_in):

        cy = cos(ang_in[2] * 0.5); #yaw
        sy = sin(ang_in[2] * 0.5); #yaw
        cp = cos(ang_in[1] * 0.5); #pitch
        sp = sin(ang_in[1] * 0.5); #pitch
        cr = cos(ang_in[0] * 0.5); #roll
        sr = sin(ang_in[0] * 0.5); #roll

        q_out = [1,0,0,0]
        q_out[0] = cy * cp * cr + sy * sp * sr;
        q_out[1] = cy * cp * sr - sy * sp * cr;
        q_out[2] = sy * cp * sr + cy * sp * cr;
        q_out[3] = sy * cp * cr - cy * sp * sr;

        return q_out




    #Function to convert rotation matrix to a quaternion
    def rotm2quat(self, R):

        tr = R[0][0]+R[1][1]+R[2][2];

        if (tr > 0):
            S = sqrt(tr+1.0) * 2; # S=4*qw 
            qw = 0.25 * S;
            qx = (R[2][1] - R[1][2]) / S;
            qy = (R[0][2] - R[2][0]) / S; 
            qz = (R[1][0] - R[0][1]) / S; 
        elif ((R[0][0] > R[1][1]) and (R[0][0] > R[2][2])):
            S = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2; # S=4*qx 
            qw = (R[2][1] - R[1][2]) / S;
            qx = 0.25 * S;
            qy = (R[0][1] + R[1][0]) / S; 
            qz = (R[0][2] + R[2][0]) / S; 
        elif (R[1][1] > R[2][2]):
            S = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2; # S=4*qy
            qw = (R[0][2] - R[2][0]) / S;
            qx = (R[0][1] + R[1][0]) / S; 
            qy = 0.25 * S;
            qz = (R[1][2] + R[2][1]) / S; 
        else:
            S = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2; # S=4*qz
            qw = (R[1][0] - R[0][1]) / S;
            qx = (R[0][2] + R[2][0]) / S;
            qy = (R[1][2] + R[2][1]) / S;
            qz = 0.25 * S;

        if(qw>0):
            q = [qw,qx,qy,qz]
        else:
            q = [-qw,-qx,-qy,-qz]

        return q




    def multiply(self,A,B):

        al = len(A)
        ac = len(A[0])
        bl = len(B)
        bc = len(B[0])

        # print ("al=%d, ac=%d, bl=%d, bc=%d" % (al,ac,bl,bc))


        C = [];
        for i in range(al):
            C.append([])
            for j in range(bc):
                n = 0
                for k in range(ac):
                    n = n + A[i][k]*B[k][j]

                # print (n)
                C[i].append(n)

        return C


    def lin(self, u):

        v = []
        for k in range(len(u)):
            v.append(u[k][0])
        return [v]



    def col(self, u):

        v = []
        for k in range(len(u)):
            v.append([u[k]])
        return v


    def normalize(self, u):

        n = 0
        for k in range(len(u)):
            n = n + u[k]**2
        n = sqrt(n)

        v = []
        for k in range(len(u)):
            v.append(u[k]/n)

        return v


    def run(self):
        """Execute the controller loop
        """
        rate = rospy.Rate(self.freq)


        pose_msg = Pose()
        odom_msg = Odometry()
        point_msg = Point()
        point_msg2 = Point()


        count1 = 0

        count2 = 0

        t_last = rospy.Time.now().to_sec()
        t_0 = rospy.Time.now().to_sec()
        count0 = 0

        count_freq = 0
        while not rospy.is_shutdown():

            # if ((rospy.Time.now().to_sec() - t_last) < 1.0/self.freq):
            #     print ("a: %f" % (rospy.Time.now().to_sec() - t_last))
            #     continue
            # print ("b")
            # t_last = rospy.Time.now().to_sec()

            # count0 = count0 + 1
            # print("\33[96mfreq: %f\33[0m" % ((count0)/(rospy.Time.now().to_sec()-t_0)))

            # t = rospy.Time.now().to_sec() - t_0
            # if(t < count_freq*(1.0/self.freq)):
            #     # print ("\ta")
            #     continue
            # count_freq = count_freq + 1
            # print ("b")
            # print (t)

            pos = [self.state[0], self.state[1], self.state[2]]
            quat_bw = [self.state[3], self.state[4], self.state[5], self.state[6]]
            vel_b = [self.state[7], self.state[8], self.state[9]]

            R_bw = self.quat2rotm(quat_bw)

            vel_w = np.matrix(R_bw)*(np.matrix(vel_b).transpose())
            vel_w = vel_w.transpose().tolist()[0]



            omega_b = [self.omega[0], self.omega[1], self.omega[2]]
            # omega_w = self.lin(self.multiply(R_bw,self.col(omega_b)))[0]
            omega_w = np.matrix(R_bw)*(np.matrix(omega_b).transpose())
            omega_w = omega_w.transpose().tolist()[0]
            # print(omega_w)


            # print (np.matrix(R_bw))
            # print (np.matrix(self.col(omega_b)))
            # print (np.matrix(omega_w))


            # print(np.matrix(R_bw).dot(np.matrix(omega_b).transpose()))
            # omega_w = omega_w.transpose().tolist()[0]

            quat_dot = self.quat_derivative(quat_bw, omega_w)


            # A = [[1,2,3],[4,5,6],[7,8,9]]
            # B = [[1,2],[1,2],[1,2]]
            # C = self.multiply(A,B)
            # print(np.matrix(A))
            # print(np.matrix(B))
            # print(np.matrix(C))

            # print("\33[96momega_b = [%f, %f, %f]\33[0m" % (self.omega[0],self.omega[1],self.omega[2]))
            # print("\33[96momega_w = [%f, %f, %f]\33[0m" % (omega_w[0],omega_w[1],omega_w[2]))
            # print("\33[96mquat_dot = [%f, %f, %f, %f]\33[0m" % (quat_dot[0],quat_dot[1],quat_dot[2],quat_dot[3]))
            # print("")

            g_vec_body = np.matrix(R_bw).transpose()*(np.matrix([0,0,9.81]).transpose())
            g_vec_body = g_vec_body.transpose().tolist()[0]

            F_drag = [-0.01*vel_b[0], -0.01*vel_b[1], -0.01*vel_b[2]]

            tau_vec = [0, 0, self.robot_m*self.tau]


            acc = [0, 0, 0]
            acc[0] = tau_vec[0]/self.robot_m - g_vec_body[0] + F_drag[0]/self.robot_m
            acc[1] = tau_vec[1]/self.robot_m - g_vec_body[1] + F_drag[1]/self.robot_m
            acc[2] = tau_vec[2]/self.robot_m - g_vec_body[2] + F_drag[2]/self.robot_m



            #Model integration
            self.state[0] = self.state[0] + vel_w[0]*(1.0/self.freq)
            self.state[1] = self.state[1] + vel_w[1]*(1.0/self.freq)
            self.state[2] = self.state[2] + vel_w[2]*(1.0/self.freq)

            # self.state[3] = self.state[3] + quat_dot[0]*(1.0/self.freq)
            # self.state[4] = self.state[4] + quat_dot[1]*(1.0/self.freq)
            # self.state[5] = self.state[5] + quat_dot[2]*(1.0/self.freq)
            # self.state[6] = self.state[6] + quat_dot[3]*(1.0/self.freq)
            q_new = [0,0,0,0]
            q_new[0] = self.state[3] + quat_dot[0]*(1.0/self.freq)
            q_new[1] = self.state[4] + quat_dot[1]*(1.0/self.freq)
            q_new[2] = self.state[5] + quat_dot[2]*(1.0/self.freq)
            q_new[3] = self.state[6] + quat_dot[3]*(1.0/self.freq)
            q_new = self.normalize(q_new)
            self.state[3] = q_new[0]
            self.state[4] = q_new[1]
            self.state[5] = q_new[2]
            self.state[6] = q_new[3]
            
            self.state[7] = self.state[7] + acc[0]*(1.0/self.freq)
            self.state[8] = self.state[8] + acc[1]*(1.0/self.freq)
            self.state[9] = self.state[9] + acc[2]*(1.0/self.freq)




            #Publis robots pose
            pose_msg.position.x = self.state[0]
            pose_msg.position.y = self.state[1]
            pose_msg.position.z = self.state[2]
            pose_msg.orientation.x = self.state[4]
            pose_msg.orientation.y = self.state[5]
            pose_msg.orientation.z = self.state[6]
            pose_msg.orientation.w = self.state[3]
            
            self.pub_pose.publish(pose_msg)



            #Publis robots odomtry (with velocity)
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "drone"
            odom_msg.child_frame_id = "world"
            odom_msg.pose.pose.position.x = self.state[0]
            odom_msg.pose.pose.position.y = self.state[1]
            odom_msg.pose.pose.position.z = self.state[2]
            odom_msg.pose.pose.orientation.x = self.state[4]
            odom_msg.pose.pose.orientation.y = self.state[5]
            odom_msg.pose.pose.orientation.z = self.state[6]
            odom_msg.pose.pose.orientation.w = self.state[3]

            odom_msg.twist.twist.linear.x = self.state[7]
            odom_msg.twist.twist.linear.y = self.state[8]
            odom_msg.twist.twist.linear.z = self.state[9]
            odom_msg.twist.twist.angular.x = self.omega[0]
            odom_msg.twist.twist.angular.y = self.omega[1]
            odom_msg.twist.twist.angular.z = self.omega[2]
            
            self.pub_odom.publish(odom_msg)



            count2 = count2 + 1
            if (count2 == 4):
                count2 = 0

                #Compute closest point - with respect to the world frame
                n_obst = min([len(self.obtscles_r), len(self.obtscles_pos)])
                D_close = float("inf")
                o_close = 0
                for o in range(n_obst):
                    Dvec = [self.state[0]-self.obtscles_pos[o][0], self.state[1]-self.obtscles_pos[o][1], self.state[2]-self.obtscles_pos[o][2]]
                    D = sqrt(Dvec[0]**2 + Dvec[1]**2 + Dvec[2]**2) - self.obtscles_r[o]
                    if (D<D_close):
                        o_close = o
                        D_close = D

                # Publish vector
                D_vec_close = [self.obtscles_pos[o_close][0]-self.state[0], self.obtscles_pos[o_close][1]-self.state[1], self.obtscles_pos[o_close][2]-self.state[2]]
                D = sqrt(D_vec_close[0]**2 + D_vec_close[1]**2 + D_vec_close[2]**2)
                D_hat = [D_vec_close[0]/(D+1e-8), D_vec_close[1]/(D+1e-8), D_vec_close[2]/(D+1e-8)]
                D = D - self.obtscles_r[o_close]
                # D_vec_close = [D_hat[0]*D, D_hat[1]*D, D_hat[2]*D]
                if D>0:
                    D_vec_close = [D_vec_close[0]-D_hat[0]*self.obtscles_r[o_close], D_vec_close[1]-D_hat[1]*self.obtscles_r[o_close], D_vec_close[2]-D_hat[2]*self.obtscles_r[o_close]]
                    close_point_world = [self.state[0] + D_hat[0]*D, self.state[1] + D_hat[1]*D, self.state[2] + D_hat[2]*D]
                else:
                    D_vec_close = [0.0,0.0,0.0]
                    close_point_world = [self.state[0], self.state[1], self.state[2]]

                # # Publish vector
                # point_msg.x = D_vec_close[0]
                # point_msg.y = D_vec_close[1]
                # point_msg.z = 0.0
                # Publish point
                point_msg.x = close_point_world[0]
                point_msg.y = close_point_world[1]
                point_msg.z = close_point_world[2]
                self.pub_closest_world.publish(point_msg)


                #Compute closest point - with respect to the robots frame

                p_cw = [[close_point_world[0]],[close_point_world[1]],[close_point_world[2]],[1]]
                # print("a")
                H_bw = self.quat2rotm(quat_bw)
                # print("b")
                H_bw[0].append(pos[0])
                H_bw[1].append(pos[1])
                H_bw[2].append(pos[2])
                H_bw.append([0,0,0,1])
                H_bw = np.matrix(H_bw)
                p_cb = H_bw**(-1)*p_cw

                point_msg2.x = p_cb[0,0]
                point_msg2.y = p_cb[1,0]
                point_msg2.z = p_cb[2,0]
                self.pub_closest_body.publish(point_msg2)
                # print ("\33[95mp_cb = [%f, %f]\33[0m" % (point_msg2.x, point_msg2.y))



                #########################################

                robot_marker_array = MarkerArray()
                
                #Publish robot marker to rviz
                marker_robot = Marker()
                marker_robot.header.frame_id = "world"
                marker_robot.header.stamp = rospy.Time.now()
                marker_robot.id = 0
                marker_robot.type = marker_robot.CUBE
                marker_robot.action = marker_robot.ADD
                marker_robot.lifetime = rospy.Duration(3)
                # Size of robot
                marker_robot.scale.x = self.robot_arm_len/2.0
                marker_robot.scale.y = self.robot_arm_len/2.0
                marker_robot.scale.z = self.robot_arm_len/6.0
                #Color of the marker
                marker_robot.color.a = 0.5
                marker_robot.color.r = 0.0
                marker_robot.color.g = 0.0
                marker_robot.color.b = 0.0
                # Position of the marker
                marker_robot.pose.position.x = self.state[0]
                marker_robot.pose.position.y = self.state[1]
                marker_robot.pose.position.z = self.state[2]
                # Orientation of the marker
                marker_robot.pose.orientation.x = self.state[4]
                marker_robot.pose.orientation.y = self.state[5]
                marker_robot.pose.orientation.z = self.state[6]
                marker_robot.pose.orientation.w = self.state[3]
                # Append marker to array
                robot_marker_array.markers.append(marker_robot)



                R0 = [[cos(pi/4),-sin(pi/4),0],[sin(pi/4),cos(pi/4),0],[0, 0, 1]]
                Rarm1 = (np.matrix(R_bw)*np.matrix(R0)).tolist()
                qarm1 = self.rotm2quat(Rarm1)
                #Publish robot marker to rviz
                marker_arm1 = Marker()
                marker_arm1.header.frame_id = "world"
                marker_arm1.header.stamp = rospy.Time.now()
                marker_arm1.id = 1
                marker_arm1.type = marker_arm1.CUBE
                marker_arm1.action = marker_arm1.ADD
                marker_arm1.lifetime = rospy.Duration(3)
                # Size of robot
                marker_arm1.scale.x = 2*self.robot_arm_len
                marker_arm1.scale.y = self.robot_arm_len/10.0
                marker_arm1.scale.z = self.robot_arm_len/10.0
                #Color of the marker
                marker_arm1.color.a = 0.9
                marker_arm1.color.r = 0.0
                marker_arm1.color.g = 0.0
                marker_arm1.color.b = 0.0
                # Position of the marker
                marker_arm1.pose.position.x = self.state[0]
                marker_arm1.pose.position.y = self.state[1]
                marker_arm1.pose.position.z = self.state[2]
                # Orientation of the marker
                # marker_arm1.pose.orientation.x = self.state[4]
                # marker_arm1.pose.orientation.y = self.state[5]
                # marker_arm1.pose.orientation.z = self.state[6]
                # marker_arm1.pose.orientation.w = self.state[3]
                marker_arm1.pose.orientation.x = qarm1[1]
                marker_arm1.pose.orientation.y = qarm1[2]
                marker_arm1.pose.orientation.z = qarm1[3]
                marker_arm1.pose.orientation.w = qarm1[0]
                # Append marker to array
                robot_marker_array.markers.append(marker_arm1)


                R0 = [[cos(pi/4),-sin(pi/4),0],[sin(pi/4),cos(pi/4),0],[0, 0, 1]]
                # R0 = [[1,0,0],[0,1,0],[0, 0, 1]]
                Rarm2 = (np.matrix(R_bw)*np.matrix(R0)).tolist()
                qarm2 = self.rotm2quat(Rarm2)
                #Publish robot marker to rviz
                marker_arm2 = Marker()
                marker_arm2.header.frame_id = "world"
                marker_arm2.header.stamp = rospy.Time.now()
                marker_arm2.id = 2
                marker_arm2.type = marker_arm2.CUBE
                marker_arm2.action = marker_arm2.ADD
                marker_arm2.lifetime = rospy.Duration(3)
                # Size of robot
                marker_arm2.scale.x = self.robot_arm_len/10.0
                marker_arm2.scale.y = 2*self.robot_arm_len
                marker_arm2.scale.z = self.robot_arm_len/10.0
                #Color of the marker
                marker_arm2.color.a = 0.9
                marker_arm2.color.r = 0.0
                marker_arm2.color.g = 0.0
                marker_arm2.color.b = 0.0
                # Position of the marker
                marker_arm2.pose.position.x = self.state[0]
                marker_arm2.pose.position.y = self.state[1]
                marker_arm2.pose.position.z = self.state[2]
                # Orientation of the marker
                marker_arm2.pose.orientation.x = qarm2[1]
                marker_arm2.pose.orientation.y = qarm2[2]
                marker_arm2.pose.orientation.z = qarm2[3]
                marker_arm2.pose.orientation.w = qarm2[0]
                # Append marker to array
                robot_marker_array.markers.append(marker_arm2)



                d_arm = 0.7071067811865476*self.robot_arm_len
                helix_d = self.robot_arm_len*0.8
                helix_h = self.robot_arm_len/15.0

                h1_pos_b = [[d_arm],[d_arm],[self.robot_arm_len/10.0]]
                h1_pos_w = (np.matrix(R_bw)*np.matrix(h1_pos_b)).transpose().tolist()[0]
                #Publish robot marker to rviz
                marker_h1 = Marker()
                marker_h1.header.frame_id = "world"
                marker_h1.header.stamp = rospy.Time.now()
                marker_h1.id = 3
                marker_h1.type = marker_h1.CYLINDER
                marker_h1.action = marker_h1.ADD
                marker_h1.lifetime = rospy.Duration(3)
                # Size of robot
                marker_h1.scale.x = helix_d
                marker_h1.scale.y = helix_d
                marker_h1.scale.z = helix_h
                #Color of the marker
                marker_h1.color.a = 0.3
                marker_h1.color.r = 0.5
                marker_h1.color.g = 0.5
                marker_h1.color.b = 0.5
                # Position of the marker
                marker_h1.pose.position.x = self.state[0] + h1_pos_w[0]
                marker_h1.pose.position.y = self.state[1] + h1_pos_w[1]
                marker_h1.pose.position.z = self.state[2] + h1_pos_w[2]
                # Orientation of the marker
                marker_h1.pose.orientation.x = self.state[4]
                marker_h1.pose.orientation.y = self.state[5]
                marker_h1.pose.orientation.z = self.state[6]
                marker_h1.pose.orientation.w = self.state[3]
                # Append marker to array
                robot_marker_array.markers.append(marker_h1)


                h2_pos_b = [[-d_arm],[d_arm],[self.robot_arm_len/10.0]]
                h2_pos_w = (np.matrix(R_bw)*np.matrix(h2_pos_b)).transpose().tolist()[0]
                #Publish robot marker to rviz
                marker_h2 = Marker()
                marker_h2.header.frame_id = "world"
                marker_h2.header.stamp = rospy.Time.now()
                marker_h2.id = 4
                marker_h2.type = marker_h2.CYLINDER
                marker_h2.action = marker_h2.ADD
                marker_h2.lifetime = rospy.Duration(3)
                # Size of robot
                marker_h2.scale.x = helix_d
                marker_h2.scale.y = helix_d
                marker_h2.scale.z = helix_h
                #Color of the marker
                marker_h2.color.a = 0.3
                marker_h2.color.r = 0.5
                marker_h2.color.g = 0.5
                marker_h2.color.b = 0.5
                # Position of the marker
                marker_h2.pose.position.x = self.state[0] + h2_pos_w[0]
                marker_h2.pose.position.y = self.state[1] + h2_pos_w[1]
                marker_h2.pose.position.z = self.state[2] + h2_pos_w[2]
                # Orientation of the marker
                marker_h2.pose.orientation.x = self.state[4]
                marker_h2.pose.orientation.y = self.state[5]
                marker_h2.pose.orientation.z = self.state[6]
                marker_h2.pose.orientation.w = self.state[3]
                # Append marker to array
                robot_marker_array.markers.append(marker_h2)


                h3_pos_b = [[-d_arm],[-d_arm],[self.robot_arm_len/10.0]]
                h3_pos_w = (np.matrix(R_bw)*np.matrix(h3_pos_b)).transpose().tolist()[0]
                #Publish robot marker to rviz
                marker_h3 = Marker()
                marker_h3.header.frame_id = "world"
                marker_h3.header.stamp = rospy.Time.now()
                marker_h3.id = 5
                marker_h3.type = marker_h3.CYLINDER
                marker_h3.action = marker_h3.ADD
                marker_h3.lifetime = rospy.Duration(3)
                # Size of robot
                marker_h3.scale.x = helix_d
                marker_h3.scale.y = helix_d
                marker_h3.scale.z = helix_h
                #Color of the marker
                marker_h3.color.a = 0.3
                marker_h3.color.r = 0.5
                marker_h3.color.g = 0.5
                marker_h3.color.b = 0.5
                # Position of the marker
                marker_h3.pose.position.x = self.state[0] + h3_pos_w[0]
                marker_h3.pose.position.y = self.state[1] + h3_pos_w[1]
                marker_h3.pose.position.z = self.state[2] + h3_pos_w[2]
                # Orientation of the marker
                marker_h3.pose.orientation.x = self.state[4]
                marker_h3.pose.orientation.y = self.state[5]
                marker_h3.pose.orientation.z = self.state[6]
                marker_h3.pose.orientation.w = self.state[3]
                # Append marker to array
                robot_marker_array.markers.append(marker_h3)

                h4_pos_b = [[d_arm],[-d_arm],[self.robot_arm_len/10.0]]
                h4_pos_w = (np.matrix(R_bw)*np.matrix(h4_pos_b)).transpose().tolist()[0]
                #Publish robot marker to rviz
                marker_h4 = Marker()
                marker_h4.header.frame_id = "world"
                marker_h4.header.stamp = rospy.Time.now()
                marker_h4.id = 6
                marker_h4.type = marker_h4.CYLINDER
                marker_h4.action = marker_h4.ADD
                marker_h4.lifetime = rospy.Duration(3)
                # Size of robot
                marker_h4.scale.x = helix_d
                marker_h4.scale.y = helix_d
                marker_h4.scale.z = helix_h
                #Color of the marker
                marker_h4.color.a = 0.3
                marker_h4.color.r = 0.5
                marker_h4.color.g = 0.5
                marker_h4.color.b = 0.5
                # Position of the marker
                marker_h4.pose.position.x = self.state[0] + h4_pos_w[0]
                marker_h4.pose.position.y = self.state[1] + h4_pos_w[1]
                marker_h4.pose.position.z = self.state[2] + h4_pos_w[2]
                # Orientation of the marker
                marker_h4.pose.orientation.x = self.state[4]
                marker_h4.pose.orientation.y = self.state[5]
                marker_h4.pose.orientation.z = self.state[6]
                marker_h4.pose.orientation.w = self.state[3]
                # Append marker to array
                robot_marker_array.markers.append(marker_h4)


                #Publish robot marker to rviz
                marker_arrow = Marker()
                marker_arrow.header.frame_id = "world"
                marker_arrow.header.stamp = rospy.Time.now()
                marker_arrow.id = 7
                marker_arrow.type = marker_arrow.ARROW
                marker_arrow.action = marker_arrow.ADD
                marker_arrow.lifetime = rospy.Duration(3)
                # Size of robot
                marker_arrow.scale.x = self.robot_arm_len*1.0
                marker_arrow.scale.y = self.robot_arm_len/5.0
                marker_arrow.scale.z = self.robot_arm_len/5.0
                #Color of the marker
                marker_arrow.color.a = 0.99
                marker_arrow.color.r = 0.0
                marker_arrow.color.g = 0.0
                marker_arrow.color.b = 1.0
                # Position of the marker
                marker_arrow.pose.position.x = self.state[0] 
                marker_arrow.pose.position.y = self.state[1]
                marker_arrow.pose.position.z = self.state[2]
                # Orientation of the marker
                marker_arrow.pose.orientation.x = self.state[4]
                marker_arrow.pose.orientation.y = self.state[5]
                marker_arrow.pose.orientation.z = self.state[6]
                marker_arrow.pose.orientation.w = self.state[3]
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
                marker_closest.scale.x = self.robot_arm_len/2.0
                marker_closest.scale.y = self.robot_arm_len/2.0
                marker_closest.scale.z = self.robot_arm_len/2.0
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

                #Publish robots history to rviz
                delta = 0.05
                approx_len = 3.0
                if(sqrt((self.state[0]-self.history[-1][0])**2+(self.state[1]-self.history[-1][1])**2+(self.state[2]-self.history[-1][2])**2) > delta):
                    self.history.append([self.state[0],self.state[1],self.state[2]])
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
                    marker.scale.x = self.robot_arm_len/3.0
                    marker.scale.y = self.robot_arm_len/3.0
                    marker.scale.z = self.robot_arm_len/3.0
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

            #end count2

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


            rate.sleep()



    def init_node(self):
        """Initialize ROS related variables, parameters and callbacks
        :return:
        """
        rospy.init_node("drone_sim")


        # parameters (description in yaml file)
        self.state = rospy.get_param("~state_0", 1.0)
        # self.robot_radius = float(rospy.get_param("~robot_radius", 1.0))
        self.robot_arm_len = float(rospy.get_param("~robot_arm_len", 1.0))
        self.robot_m = float(rospy.get_param("~robot_m", 1.0))
        # self.robot_width = float(rospy.get_param("~robot_width", 1.0))
        # self.robot_height = float(rospy.get_param("~robot_height", 1.0))
        # self.robot_a = float(rospy.get_param("~robot_a", 1.0))
        # self.robot_b = float(rospy.get_param("~robot_b", 1.0))
        # self.robot_r = float(rospy.get_param("~robot_r", 1.0))
        self.obtscles_pos = rospy.get_param("~obtscles_pos", [])
        self.obtscles_r = rospy.get_param("~obtscles_r", [])
        self.history = []
        self.history.append([self.state[0], self.state[1], self.state[2]])

        print("self.state: ", self.state)
        print("self.robot_arm_len: ", self.robot_arm_len)
        print("self.robot_m: ", self.robot_m)

        # publishers
        self.pub_pose = rospy.Publisher("/drone/pose", Pose, queue_size=1)
        self.pub_odom = rospy.Publisher("/drone/odom", Odometry, queue_size=1)
        self.pub_closest_world = rospy.Publisher("/drone/closest_point_world", Point, queue_size=1)
        self.pub_closest_body = rospy.Publisher("/drone/closest_point_body", Point, queue_size=1)
        self.pub_rviz_robot = rospy.Publisher("/drone/robot", MarkerArray, queue_size=1)
        self.pub_rviz_closest = rospy.Publisher("/drone/closest_marker", Marker, queue_size=1)
        self.pub_rviz_obst = rospy.Publisher("/drone/obstacles", MarkerArray, queue_size=1)
        self.pub_rviz_hist = rospy.Publisher("/drone/history", MarkerArray, queue_size=1)


        # self.pub_rviz_robot_w1 = rospy.Publisher("/drone/robot_w1", Marker, queue_size=1)

        # subscribers
        rospy.Subscriber("/drone/acrorate", Quaternion, self.callback_acrorate)
        # rospy.Subscriber("/drone/wheels_speeds", Float32MultiArray, self.callback_wheels)




    def callback_acrorate(self, data):
        """Callback to get the reference velocity for the robot
        :param data: pose ROS message
        """
        self.tau = data.w
        self.omega = [data.x, data.y, data.z]





    def callback_wheels(self, data):
        """Callback to get the reference velocity for the robot
        :param data: pose ROS message
        """
        vr = (data.data[0] + data.data[1])/2.0
        vl = (data.data[2] + data.data[3])/2.0

        vx = 0.5*(vr+vl)
        wz = (self.robot_b/(2*(self.robot_a**2+self.robot_b**2)))*(vr-vl)
        # wz = 0

        vel = [vx, wz]
        # print ("\33[96m[vx, wz] = [%f, %f]\33[0m" % (vx, wz))


        self.vel = vel



if __name__ == '__main__':
    node = drone_node()
    node.run()
