#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from enum import Enum
from itertools import groupby
import numpy as np
from math import pi, sqrt, cos, sin, tan, acos, asin, atan, atan2

#import distancefield
from distancefield.distancefield_class import distancefield_class
import math_utils.math_utils as MU


class quadrobot_class():


    def __init__(self, vr, kf, reverse_direction, flag_follow_obstacle, epsilon, switch_dist_0, switch_dist, m, Kv, Kw):

        # base variables
        self.state = [0, 0, 0]

        #Obstacle follower parameters
        self.epsilon = epsilon
        self.switch_dist_0 = switch_dist_0
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

        self.D_hist = 1000 #temp

        #Vector field object
        self.vec_field_obj = distancefield_class(vr, kf, reverse_direction, self.flag_follow_obstacle, self.epsilon, self.switch_dist_0, self.switch_dist)



    #Compute the Jacobian of the vector field
    def compute_Jacobian(self, pos):

        J = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        delta = 0.0001

        px = [pos[0]+delta,pos[1],pos[2]]
        py = [pos[0],pos[1]+delta,pos[2]]
        pz = [pos[0],pos[1],pos[2]+delta]

        Vx,Vy,Vz,flag = self.vec_field_obj.compute_field_at_p(pos)
        f0 = [Vx,Vy,Vz]
        Vx,Vy,Vz,flag = self.vec_field_obj.compute_field_at_p(px)
        fx = [Vx,Vy,Vz]
        Vx,Vy,Vz,flag = self.vec_field_obj.compute_field_at_p(py)
        fy = [Vx,Vy,Vz]
        Vx,Vy,Vz,flag = self.vec_field_obj.compute_field_at_p(pz)
        fz = [Vx,Vy,Vz]

        #print("f0:", f0)
        #print("fx:", fx)
        #print("fy:", fy)
        #print("fz:", fz)

        J[0][0] = (fx[0]-f0[0])/delta; J[0][1] = (fy[0]-f0[0])/delta; J[0][2] = (fz[0]-f0[0])/delta
        J[1][0] = (fx[1]-f0[1])/delta; J[1][1] = (fy[1]-f0[1])/delta; J[1][2] = (fz[1]-f0[1])/delta
        J[2][0] = (fx[2]-f0[2])/delta; J[2][1] = (fy[2]-f0[2])/delta; J[2][2] = (fz[2]-f0[2])/delta

        #print(J)
        #print('')
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
    def get_acc_ref(self, pos, vel, out=[0], id=0):

        Vx, Vy, Vz, flag = self.vec_field_obj.compute_field_at_p(pos)
        f = [Vx, Vy, Vz]
        J = self.compute_Jacobian(pos)

        a_r = [0,0,0]
        a_r[0] = J[0][0]*vel[0] + J[0][1]*vel[1] + J[0][2]*vel[2] + self.kv*(f[0]-vel[0])
        a_r[1] = J[1][0]*vel[0] + J[1][1]*vel[1] + J[1][2]*vel[2] + self.kv*(f[1]-vel[1])
        a_r[2] = J[2][0]*vel[0] + J[2][1]*vel[1] + J[2][2]*vel[2] + self.kv*(f[2]-vel[2]) + self.g

        out[id] = a_r

        return a_r



    def control_step(self):

        delta_t = 0.01

        pos = [self.state[0], self.state[1], self.state[2]]
        quat = [self.state[3], self.state[4], self.state[5], self.state[6]]
        vel = [self.state[7], self.state[8], self.state[9]]

        R = MU.quat2rotm(quat)


        z_b = [R[0][2], R[1][2], R[2][2]]
        z_hat = [0, 0, 1]

        Vx, Vy, Vz, flag = self.vec_field_obj.compute_field_at_p(pos)
        f = [Vx, Vy, Vz]

        pos_M = [pos[0]+vel[0]*delta_t, pos[1]+vel[1]*delta_t, pos[2]+vel[2]*delta_t]
        pos_m = [pos[0]-vel[0]*delta_t, pos[1]-vel[1]*delta_t, pos[2]-vel[2]*delta_t]
        VxM, VyM, VzM, flag = self.vec_field_obj.compute_field_at_p(pos_M)
        Vxm, Vym, Vzm, flag = self.vec_field_obj.compute_field_at_p(pos_m)

        psi_r = atan2(Vy,Vx)
        psi_r_M = atan2(VyM,VxM)
        psi_r_m = atan2(Vym,Vxm)
        # psi_r = 0
        # psi_r_M = 0
        # psi_r_m = 0

        # Computation of reference orientation
        a_r = self.get_acc_ref(pos,vel)
        Rr =  self.get_orientation_ref(a_r, psi_r)

        #print ("ar: ", a_r)
        #print ("z_b: ", z_b)
        dot_ar_zb = a_r[0]*z_b[0] + a_r[1]*z_b[1] + a_r[2]*z_b[2]
        self.tau = self.m*dot_ar_zb

        Re = np.matrix(R).transpose()*np.matrix(Rr)
        Re = Re.tolist()

        # Compute the time derivative of Rr
        ####################
        tau_r = self.m*self.g

        pos_M = [pos[0] + vel[0]*delta_t, pos[1] + vel[1]*delta_t, pos[2] + vel[2]*delta_t]
        vel_M = [0,0,0]
        vel_M[0] = vel[0] + (z_b[0]*tau_r/self.m - self.g*z_hat[0])*delta_t
        vel_M[1] = vel[1] + (z_b[1]*tau_r/self.m - self.g*z_hat[1])*delta_t
        vel_M[2] = vel[2] + (z_b[2]*tau_r/self.m - self.g*z_hat[2])*delta_t
        a_r_M = self.get_acc_ref(pos_M,vel_M)
        Rr_M =  self.get_orientation_ref(a_r_M, psi_r_M)

        pos_m = [pos[0] - vel[0]*delta_t, pos[1] - vel[1]*delta_t, pos[2] - vel[2]*delta_t]
        vel_m = [0,0,0]
        vel_m[0] = vel[0] - (z_b[0]*tau_r/self.m - self.g*z_hat[0])*delta_t
        vel_m[1] = vel[1] - (z_b[1]*tau_r/self.m - self.g*z_hat[1])*delta_t
        vel_m[2] = vel[2] - (z_b[2]*tau_r/self.m - self.g*z_hat[2])*delta_t
        a_r_m = self.get_acc_ref(pos_m,vel_m)
        Rr_m =  self.get_orientation_ref(a_r_m, psi_r_m)

        Rr_dot = ((np.matrix(Rr_M)-np.matrix(Rr_m))/(2*delta_t)).tolist()

        S_w = np.matrix(R).transpose()*np.matrix(Rr_dot)
        S_w = S_w*(np.matrix(Re).transpose())
        S_w = S_w.tolist()
        ####################

        omega_d = [S_w[2][1]-S_w[1][2], S_w[0][2]-S_w[2][0], S_w[1][0]-S_w[0][1]] #?
        omega_d = [omega_d[0]/2.0, omega_d[1]/2.0, omega_d[2]/2.0] #?
        #omega_d = [0,0,0]

        axis, alpha = MU.rotm2axang(Re)

        omega = [0,0,0]
        omega[0] = omega_d[0] + self.kw*sin(alpha)*axis[0]
        omega[1] = omega_d[1] + self.kw*sin(alpha)*axis[1]
        omega[2] = omega_d[2] + self.kw*sin(alpha)*axis[2]

        self.omega = [omega[0], omega[1], omega[2]]

        # print ("pos: [%f, %f, %f]" % (pos[0], pos[1], pos[2]))
        # print ("quat: [%f, %f, %f, %f]" % (quat[0], quat[1], quat[2], quat[3]))
        # print ("vel: [%f, %f, %f]" % (vel[0], vel[1], vel[2]))
        # print ("f: [%f, %f, %f]" % (f[0], f[1], f[2]))
        # print ("a_r: [%f, %f, %f]" % (a_r[0], a_r[1], a_r[2]))
        # print ("self.tau: %f" % (self.tau))
        # print (np.matrix(Rr))
        # print("")


        #Temporaryly induce error to debug
        if(self.tau<0):
            tau = 1
            print("\33[93m[Warning] tau<0\33[0m")
            # a = 1/0


    def get_acrorate(self):
        return self.tau, self.omega


    def set_state(self, state):
        self.state = state

        self.vec_field_obj.set_pos([state[0], state[1], state[2]])


    def set_pos(self, pos):
        self.state[0] = pos[0]
        self.state[1] = pos[1]
        self.state[2] = pos[2]
        self.vec_field_obj.set_pos([pos[0], pos[1], pos[2]])

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
