#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from enum import Enum
from itertools import groupby
import numpy as np
from math import pi, sqrt, cos, sin, tan, acos, asin, atan

import distancefield
# import distancefield_class
from distancefield.distancefield_class import distancefield_class
# import distancefield.distancefield_class

class quadrobot_class():


    def __init__(self, vr, kf, reverse_direction, flag_follow_obstacle, epsilon, switch_dist, m, Kv, Kw):

        # base variables
        self.state = [0, 0, 0]

        #Obstacle follower parameters
        self.epsilon = epsilon
        self.switch_dist = switch_dist
        self.closest_world = [0,0,0]
        self.flag_follow_obstacle = flag_follow_obstacle

        # controller constants
        self.m = m
        self.kv = Kv
        self.kw = Kw
        self.g = 9.81

        #Commands
        self.tau = self.m*self.g
        self.omega = [0.0, 0.0, 0.0]

        # flags
        # self.is_forward_motion_flag = is_forward_motion_flag
        # self.flag_follow_obstacle = flag_follow_obstacle
        # self.closed_path_flag = False
        # self.reverse_direction = reverse_direction

        # # obstacle avoidance point information
        # self.delta_m = 1000.0  # minimum distance
        # self.phi_m = 0.0  # angle minimum distance (body frame)

        self.D_hist = 1000 #temp

        #Vector field object
        self.vec_field_obj = distancefield_class(vr, kf, reverse_direction, self.flag_follow_obstacle, self.epsilon, self.switch_dist)






    

    #Compute the Jacobian of the vector field
    def compute_Jacobian(self, pos):

        J = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        delta = 0.0001;

        px = [pos[0]+delta,pos[1],pos[2]];
        py = [pos[0],pos[1]+delta,pos[2]];
        pz = [pos[0],pos[1],pos[2]+delta];

        Vx,Vy,Vz,flag = self.vec_field_obj.compute_field_at_p(pos)
        f0 = [Vx,Vy,Vz]
        Vx,Vy,Vz,flag = self.vec_field_obj.compute_field_at_p(px)
        fx = [Vx,Vy,Vz]
        Vx,Vy,Vz,flag = self.vec_field_obj.compute_field_at_p(py)
        fy = [Vx,Vy,Vz]
        Vx,Vy,Vz,flag = self.vec_field_obj.compute_field_at_p(pz)
        fz = [Vx,Vy,Vz]


        # J[0][0] = (fx[0]-f0[0])/delta; J[0][1] = (fx[1]-f0[1])/delta; J[0][2] = (fx[2]-f0[2])/delta;
        # J[1][0] = (fy[0]-f0[0])/delta; J[1][1] = (fy[1]-f0[1])/delta; J[1][2] = (fy[2]-f0[2])/delta;
        # J[2][0] = (fz[0]-f0[0])/delta; J[2][1] = (fz[1]-f0[1])/delta; J[2][2] = (fz[2]-f0[2])/delta;

        J[0][0] = (fx[0]-f0[0])/delta; J[0][1] = (fy[0]-f0[0])/delta; J[0][2] = (fz[0]-f0[0])/delta;
        J[1][0] = (fx[1]-f0[1])/delta; J[1][1] = (fy[1]-f0[1])/delta; J[1][2] = (fz[1]-f0[1])/delta;
        J[2][0] = (fx[2]-f0[2])/delta; J[2][1] = (fy[2]-f0[2])/delta; J[2][2] = (fz[2]-f0[2])/delta;

        return J


    # Compute a reference oientation matrix
    def get_orientation_ref(self, a_r, psi_r):

        ar_norm = sqrt(a_r[0]**2 + a_r[1]**2 + a_r[2]**2) + 0.00000000001
        z_r = [a_r[0]/ar_norm, a_r[1]/ar_norm, a_r[2]/ar_norm]

        w_psi = [cos(psi_r), sin(psi_r), 0]

        dot_w_z = w_psi[0]*z_r[0] + w_psi[1]*z_r[1] + w_psi[2]*z_r[2]

        x_r = [w_psi[0]-dot_w_z*z_r[0], w_psi[1]-dot_w_z*z_r[1], w_psi[2]-dot_w_z*z_r[2]]
        x_r_norm = sqrt(x_r[0]*x_r[0] + x_r[1]*x_r[1] + x_r[2]*x_r[2]) + 0.00000000001
        x_r = [x_r[0]/x_r_norm, x_r[1]/x_r_norm, x_r[2]/x_r_norm]

        y_r = [z_r[1]*x_r[2]-z_r[2]*x_r[1], z_r[2]*x_r[0]-z_r[0]*x_r[2], z_r[0]*x_r[1]-z_r[1]*x_r[0]]

        Rr = [[x_r[0],y_r[0],z_r[0]],[x_r[1],y_r[1],z_r[1]],[x_r[2],y_r[2],z_r[2]]]

        return Rr




    # Compute a reference acceleration
    def get_acc_ref(self, pos, vel):
        
        Vx, Vy, Vz, flag = self.vec_field_obj.compute_field_at_p(pos)
        f = [Vx, Vy, Vz]
        J = self.compute_Jacobian(pos)



        a_r = [0,0,0]
        a_r[0] = J[0][0]*vel[0] + J[0][1]*vel[1] + J[0][2]*vel[2] + self.kv*(f[0]-vel[0])
        a_r[1] = J[1][0]*vel[0] + J[1][1]*vel[1] + J[1][2]*vel[2] + self.kv*(f[1]-vel[1])
        a_r[2] = J[2][0]*vel[0] + J[2][1]*vel[1] + J[2][2]*vel[2] + self.kv*(f[2]-vel[2]) + self.g

        # print ("\33[96m")
        # print (np.matrix(f))
        # print (np.matrix(J))
        # print (np.matrix(a_r))
        # print ("\33[0m")

        return a_r





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



    def control_step(self):

        delta_t = 0.01

        pos = [self.state[0], self.state[1], self.state[2]]
        quat = [self.state[3], self.state[4], self.state[5], self.state[6]]
        vel = [self.state[7], self.state[8], self.state[9]]

        R = self.quat2rotm(quat)


        z_b = [R[0][2], R[1][2], R[2][2]];
        z_hat = [0, 0, 1];

        psi_r = 0
        psi_r_M = 0
        psi_r_m = 0


        Vx, Vy, Vz, flag = self.vec_field_obj.compute_field_at_p(pos)
        f = [Vx, Vy, Vz]

        # Computation of reference orientation
        a_r = self.get_acc_ref(pos,vel);
        # Matrix3d Rr, Rr_M, Rr_m;
        Rr =  self.get_orientation_ref(a_r, psi_r);

        # print(np.matrix(Rr))
        # print("")


        dot_ar_zb = a_r[0]*z_b[0] + a_r[1]*z_b[1] + a_r[2]*z_b[2]
        self.tau = self.m*dot_ar_zb;



        Re = np.matrix(R).transpose()*np.matrix(Rr);
        Re = Re.tolist()

        # Compute the time derivative of Rr
        ####################
        pos_M = [pos[0] + vel[0]*delta_t, pos[1] + vel[1]*delta_t, pos[2] + vel[2]*delta_t]
        vel_M = [0,0,0]
        vel_M[0] = vel[0] + (z_b[0]*self.tau/self.m - self.g*z_hat[0])*delta_t
        vel_M[1] = vel[1] + (z_b[1]*self.tau/self.m - self.g*z_hat[1])*delta_t
        vel_M[2] = vel[2] + (z_b[2]*self.tau/self.m - self.g*z_hat[2])*delta_t
        a_r_M = self.get_acc_ref(pos_M,vel_M);
        Rr_M =  self.get_orientation_ref(a_r_M, psi_r_M);

        pos_m = [pos[0] - vel[0]*delta_t, pos[1] - vel[1]*delta_t, pos[2] - vel[2]*delta_t]
        vel_m = [0,0,0]
        vel_m[0] = vel[0] - (z_b[0]*self.tau/self.m - self.g*z_hat[0])*delta_t
        vel_m[1] = vel[1] - (z_b[1]*self.tau/self.m - self.g*z_hat[1])*delta_t
        vel_m[2] = vel[2] - (z_b[2]*self.tau/self.m - self.g*z_hat[2])*delta_t
        a_r_m = self.get_acc_ref(pos_m,vel_m);
        Rr_m =  self.get_orientation_ref(a_r_m, psi_r_m);

        Rr_dot = ((np.matrix(Rr_M)-np.matrix(Rr_m))/(2*delta_t)).tolist()

        S_w = np.matrix(R).transpose()*np.matrix(Rr_dot)
        S_w = S_w*(np.matrix(Re).transpose())
        S_w = S_w.tolist()
        ####################

        omega_d = [S_w[2][1]-S_w[1][2], S_w[0][2]-S_w[2][0], S_w[1][0]-S_w[0][1]]
        omega_d = [omega_d[0]/2.0, omega_d[1]/2.0, omega_d[2]/2.0]
        # print ("omega_d: [%f, %f, %f]" % (omega_d[0],omega_d[1],omega_d[2]))
        # omega_d = [0,0,0]


        axis, alpha = self.rotm2axang(Re)


        omega = [0,0,0]
        omega[0] = omega_d[0] + self.kw*sin(alpha)*axis[0]
        omega[1] = omega_d[1] + self.kw*sin(alpha)*axis[1]
        omega[2] = omega_d[2] + self.kw*sin(alpha)*axis[2]

        # print ("omega:   [%f, %f, %f]" % (omega[0],omega[1],omega[2]))

        self.omega = [omega[0], omega[1], omega[2]]



        # print ("pos: [%f, %f, %f]" % (pos[0], pos[1], pos[2]))
        # print ("quat: [%f, %f, %f, %f]" % (quat[0], quat[1], quat[2], quat[3]))
        # print ("vel: [%f, %f, %f]" % (vel[0], vel[1], vel[2]))
        # print ("f: [%f, %f, %f]" % (f[0], f[1], f[2]))
        # print ("a_r: [%f, %f, %f]" % (a_r[0], a_r[1], a_r[2]))
        # print ("self.tau: %f" % (self.tau))
        # print (np.matrix(Rr))
        # print("")










        if(self.tau<0):
            a = 1/0

        # J = self.compute_Jacobian(pos)
        # Rr = self.get_orientation_ref([0,3,10],0)


        #end




















    def set_state(self, state):
        self.state = state
        
        # print("state: ")
        # print(state)
        self.vec_field_obj.set_pos([state[0], state[1], state[2]])


    def set_pos(self, pos):
        self.state[0] = pos[0]
        self.state[1] = pos[1]
        self.state[2] = pos[2]

    def set_quat(self, quat):
        self.state[3] = quat[0]
        self.state[4] = quat[1]
        self.state[5] = quat[2]
        self.state[6] = quat[3]

    def set_vel(self, vel):
        self.state[7] = vel[0]
        self.state[8] = vel[1]
        self.state[9] = vel[2]

    def set_closest(self, point):
        self.closest_world = point


    def get_acrorate(self):
        return self.tau, self.omega



    def quat2axang(self, q):

        if(q[0]<0):
            q = [-q[0], -q[1], -q[2], -q[3]]

        s = sqrt(q[1]**2+q[2]**2+q[3]**2) + 0.000001
        axis = [q[1]/s, q[2]/s, q[3]/s]

        ang = 2 * acos(q[0])

        return axis, ang


    def rotm2axang(self, R):


        q = self.rotm2quat(R)

        axis, ang = self.quat2axang(q)

        return axis, ang

       


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


