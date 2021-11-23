#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from enum import Enum
from itertools import groupby


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
        self.Kv = Kv
        self.Kw = Kw

        # flags
        # self.is_forward_motion_flag = is_forward_motion_flag
        # self.flag_follow_obstacle = flag_follow_obstacle
        # self.closed_path_flag = False
        self.reverse_direction = reverse_direction

        # # obstacle avoidance point information
        # self.delta_m = 1000.0  # minimum distance
        # self.phi_m = 0.0  # angle minimum distance (body frame)

        self.D_hist = 1000 #temp

        #Vector field object
        self.vec_field_obj = distancefield_class(vr, kf, reverse_direction, self.flag_follow_obstacle, self.epsilon, self.switch_dist)



    def set_state(self, state):
        self.state = state
        
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

