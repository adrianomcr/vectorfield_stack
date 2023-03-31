#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from enum import Enum
from itertools import groupby

from distancefield.distancefield_class import distancefield_class

class groundrobot_class():


    def __init__(self, vr, kf, reverse_direction, flag_follow_obstacle, epsilon, switch_dist_0, switch_dist, d, move_backwards):

        # base variables
        self.state = [0, 0, 0]

        #Obstacle follower parameters
        self.epsilon = epsilon
        self.switch_dist_0 = switch_dist_0
        self.switch_dist = switch_dist
        self.closest_world = [0,0,0]
        self.flag_follow_obstacle = flag_follow_obstacle

        # controller constants
        self.d_feedback = d

        # flags
        self.reverse_direction = reverse_direction
        self.move_backwards = move_backwards

        self.D_hist = 1000 #temp

        #Vector field object
        self.vec_field_obj = distancefield_class(vr, kf, reverse_direction, self.flag_follow_obstacle, self.epsilon, self.switch_dist_0, self.switch_dist)





    def set_state(self, state):
        self.state = state

        #For the vector field, consider the distance of the control point
        if self.move_backwards:
            d_pos = [self.state[0] - self.d_feedback*math.cos(self.state[2]), self.state[1] - self.d_feedback*math.sin(self.state[2])]
        else:
            d_pos = [self.state[0] + self.d_feedback*math.cos(self.state[2]), self.state[1] + self.d_feedback*math.sin(self.state[2])]
        
        self.vec_field_obj.set_pos([d_pos[0], d_pos[1], 0.0])


    def set_closest(self, point):
        self.closest_world = point

    def get_vw(self):
        [fx,fy,fz,terminate] = self.vec_field_obj.vec_field_path()
        f = [fx,fy]
        [v, w] = self.feedback_linearization(f)

        return v, w

    def feedback_linearization(self,f):
        psi = self.state[2]

        v_x = math.cos(psi) * f[0] + math.sin(psi) * f[1]
        omega_z = (-math.sin(psi) / self.d_feedback) * f[0] + (math.cos(psi) / self.d_feedback) * f[1]

        if(self.move_backwards):
            omega_z = -omega_z

        return v_x, omega_z


    def get_wheels_diferential(self,b):

        [vr, vl] = self.get_wheels_skidsteer(0,b)
        return vr, vl


    def get_wheels_skidsteer(self, a, b):

        [vx, wz] = self.get_vw()
        vr = 1.0*vx + ((a**2 + b**2)/b)*wz
        vl = 1.0*vx - ((a**2 + b**2)/b)*wz

        return vr, vl


