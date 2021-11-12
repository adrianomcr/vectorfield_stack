#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from enum import Enum
from itertools import groupby


import distancefield
# import distancefield_class
from distancefield.distancefield_class import distancefield_class
# import distancefield.distancefield_class

class groundrobot_class():


    def __init__(self, vr, kf, reverse_direction, flag_follow_obstacle, epsilon, switch_dist, d, move_backwards):

        # base variables
        self.state = [0, 0, 0]

        #Obstacle follower parameters
        self.epsilon = epsilon
        self.switch_dist = switch_dist
        self.closest = [0,0,0]
        self.flag_follow_obstacle = flag_follow_obstacle

        # controller constants
        self.d_feedback = d

        # flags
        # self.is_forward_motion_flag = is_forward_motion_flag
        # self.flag_follow_obstacle = flag_follow_obstacle
        # self.closed_path_flag = False
        self.reverse_direction = reverse_direction
        self.move_backwards = move_backwards

        # # obstacle avoidance point information
        # self.delta_m = 1000.0  # minimum distance
        # self.phi_m = 0.0  # angle minimum distance (body frame)

        self.D_hist = 1000 #temp

        #Vector field object
        self.vec_field_obj = distancefield_class(vr, kf, reverse_direction, False, 0.5, 1.0)



    def get_vw(self):
        [fx,fy,fz,terminate] = self.vec_field_obj.vec_field_path()
        f = [fx,fy]
        [v, w] = self.feedback_linearization(f)

        return v, w

    def feedback_linearization(self,f):
        psi = self.state[2]

        print("\33[96mself.state: %f, %f, %f\33[0m" % (self.state[0], self.state[1], self.state[2]))

        v_x = math.cos(psi) * f[0] + math.sin(psi) * f[1]
        omega_z = (-math.sin(psi) / self.d_feedback) * f[0] + (math.cos(psi) / self.d_feedback) * f[1]

        if(self.move_backwards):
            omega_z = -omega_z

        return v_x, omega_z


    def get_wheels_diferential(self,b,r):

        [vr, vl] = get_wheels_skidsteer(0,b,r)
        return vr, vl


    def get_wheels_skidsteer(self, a, b, r):
        vr = 0
        vl = 0
        return vr, vl


    def set_state(self, state):
        self.state = state

        d_pos = [self.state[0] + self.d_feedback*math.cos(self.state[2]), self.state[1] + self.d_feedback*math.sin(self.state[2])]
        self.vec_field_obj.set_pos([d_pos[0], d_pos[1], 0.0])
        # print("\33[96mset_state\33[0m")


    # def set_closest(self, point):
    #     self.closest = point
    #     self.vec_field_obj.set_pos([self.state[0], self.state[1], 0.0])


    # def set_trajectory(self, traj, insert_n_points, filter_path_n_average, closed_path_flag):
    #     """Callback to obtain the trajectory to be followed by the robot
    #     :param data: trajectory ROS message
    #     """
    #     # self.reset()

    #     # remove consecutive points and check for repeated points at the start and tail
    #     traj = [x[0] for x in groupby(traj)]
    #     if len(traj) > 1:
    #         if traj[0] == traj[-1]:
    #             traj.pop(-1)

    #     self.traj = traj
    #     self.closed_path_flag = closed_path_flag

    #     # Insert points on the path
    #     if insert_n_points > 0:
    #         self.traj = self.insert_points(self.traj, insert_n_points, closed_path_flag)

    #     # Filter the points (average filter)
    #     if filter_path_n_average > 0:
    #         self.traj = self.filter_path(self.traj, filter_path_n_average, closed_path_flag)

    #     # Update the closest index - index to the closest point in the curve
    #     self.state_k = 0
    #     d = float("inf")
    #     for k in range(len(self.traj)):
    #         d_temp = math.sqrt((self.pos[0] - self.traj[k][0]) ** 2 + (self.pos[1] - self.traj[k][1]) ** 2 + (self.pos[2] - self.traj[k][2]) ** 2)
    #         if d_temp < d:
    #             self.state_k = k
    #             d = d_temp

    #     rospy.loginfo("New path received by the controller (%d points)", len(self.traj))

