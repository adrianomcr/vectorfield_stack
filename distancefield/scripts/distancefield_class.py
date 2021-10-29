#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from enum import Enum
from itertools import groupby




class distancefield_class():


    # def __init__(self, v_r, k_f, epsilon, switch_dist, is_forward_motion_flag, flag_follow_obstacle):
    def __init__(self, v_r, k_f, reverse_direction, flag_follow_obstacle, epsilon, switch_dist):

        # base variables
        self.pos = [0, 0, 0]
        #self.rpy = [0, 0, 0]
        #self.quat = [1, 0, 0, 0]
        self.traj = []
        self.state_k = 0
        self.state_k_delta = 10

        #Obstacle follower parameters
        self.epsilon = epsilon
        self.switch_dist = switch_dist
        self.closest = [0,0,0]
        self.flag_follow_obstacle = flag_follow_obstacle

        # controller constants
        self.v_r = v_r
        self.k_f = k_f
        # self.d_feedback = d_feedback

        # flags
        # self.is_forward_motion_flag = is_forward_motion_flag
        # self.flag_follow_obstacle = flag_follow_obstacle
        # self.closed_path_flag = False
        self.reverse_direction = reverse_direction

        # # obstacle avoidance point information
        # self.delta_m = 1000.0  # minimum distance
        # self.phi_m = 0.0  # angle minimum distance (body frame)

        self.D_hist = 1000 #temp


    # def __init__(self, v_r, k_f, reverse_direction):

    #     # base variables
    #     self.pos = [0, 0, 0]
    #     #self.rpy = [0, 0, 0]
    #     #self.quat = [1, 0, 0, 0]
    #     self.traj = []
    #     self.state_k = 0
    #     self.state_k_delta = 10

    #     #Obstacle follower parameters
    #     self.flag_follow_obstacle = False

    #     # controller constants
    #     self.v_r = v_r
    #     self.k_f = k_f
    #     # self.d_feedback = d_feedback

    #     # flags
    #     # self.is_forward_motion_flag = is_forward_motion_flag
    #     # self.flag_follow_obstacle = flag_follow_obstacle
    #     # self.closed_path_flag = False
    #     self.reverse_direction = reverse_direction




    # def set_pos(self, pos, rpy):
    #def set_pos(self, pos, quat):
    def set_pos(self, pos):
        self.pos = pos
        # self.rpy = rpy
        # self.quat = quat

    def set_closest(self, point):
        self.closest = point
        # self.rpy = rpy
        # self.quat = quat





    def is_ready(self):
        return True if self.traj and len(self.traj) > 0 else False


    def set_trajectory(self, traj, insert_n_points, filter_path_n_average, closed_path_flag):
        """Callback to obtain the trajectory to be followed by the robot
        :param data: trajectory ROS message
        """
        # self.reset()

        # remove consecutive points and check for repeated points at the start and tail
        traj = [x[0] for x in groupby(traj)]
        if len(traj) > 1:
            if traj[0] == traj[-1]:
                traj.pop(-1)

        self.traj = traj
        self.closed_path_flag = closed_path_flag

        # Insert points on the path
        if insert_n_points > 0:
            self.traj = self.insert_points(self.traj, insert_n_points, closed_path_flag)

        # Filter the points (average filter)
        if filter_path_n_average > 0:
            self.traj = self.filter_path(self.traj, filter_path_n_average, closed_path_flag)

        # Update the closest index - index to the closest point in the curve
        self.state_k = 0
        d = float("inf")
        for k in range(len(self.traj)):
            d_temp = math.sqrt((self.pos[0] - self.traj[k][0]) ** 2 + (self.pos[1] - self.traj[k][1]) ** 2 + (self.pos[2] - self.traj[k][2]) ** 2)
            if d_temp < d:
                self.state_k = k
                d = d_temp

        rospy.loginfo("New path received by the controller (%d points)", len(self.traj))



    def vec_field_path(self):
            """Compute the vector field that will guide the robot through a path
            :return:
                Vx, Vy, reached_endpoint
            """
            x, y, z = self.pos
            local_traj = self.traj
            size_traj = len(local_traj)
            reached_endpoint = False

            # Compute the closest ponit on the curve
            # Consider only the points in the vicinity of the current closest point (robustness)
            k_vec = [self.state_k - self.state_k_delta + i for i in range(self.state_k_delta)]
            k_vec.append(self.state_k)
            k_vec = k_vec + [self.state_k + 1 + i for i in range(self.state_k_delta)]
            for k in range(len(k_vec)):
                if k_vec[k] < 0:
                    k_vec[k] = k_vec[k] + size_traj
                if k_vec[k] >= size_traj:
                    k_vec[k] = k_vec[k] - size_traj


            # iterate over the k_vec indices to get the closest point
            D = float("inf")
            k_min = size_traj
            for k in k_vec:
                D_temp = math.sqrt((x - local_traj[k][0]) ** 2 + (y - local_traj[k][1]) ** 2 + (z - local_traj[k][2]) ** 2)
                if D_temp < D:
                    k_min = k
                    D = D_temp
            self.state_k = k_min  # index of the closest point

            # print ("self.state_k: ", self.state_k)

            # compute the distance vector
            D_vec = [x - local_traj[k_min][0], y - local_traj[k_min][1], z - local_traj[k_min][2]]
            # print("self.pos: ", self.pos)
            # print("x, y, z: ", x, y, z)
            # print("D_vec: ", D_vec)
            # print("local_traj[k][2]: ", local_traj[k][2])
            # print("")
            # compute the gradient of the distance Function
            grad_D = [D_vec[0] / (D + 0.000001), D_vec[1] / (D + 0.000001), D_vec[2] / (D + 0.000001)]

            # compute neighbors of k_min
            k1 = k_min - 1  # previous index
            k2 = k_min + 1  # next index
            if self.closed_path_flag:
                # consider that the first point and the last are neighbors
                if k1 == -1:
                    k1 = size_traj - 1
                if k2 == size_traj:
                    k2 = 0
            else:
                # consider that the first point and the last are distant apart
                if k1 == -1:
                    k1 = 0
                if k2 == size_traj:
                    k2 = size_traj - 1

            # numerically compute the tangent vector and normalize it
            T = [local_traj[k2][0] - local_traj[k1][0], local_traj[k2][1] - local_traj[k1][1], local_traj[k2][2] - local_traj[k1][2]]
            norm_T = math.sqrt(T[0] ** 2 + T[1] ** 2 + T[2] ** 2) + 0.000001
            
            #Possibly invert the direcion that the curve is followed
            if(self.reverse_direction):
                T = [-T[0] / norm_T, -T[1] / norm_T, -T[2] / norm_T]
            else:
                T = [T[0] / norm_T, T[1] / norm_T, T[2] / norm_T]


            # Gain functions
            G = -(2 / math.pi) * math.atan(self.k_f * D)  # convergence
            H = math.sqrt(1 - G ** 2)  # circulation

            # compute the field's components
            Vx = self.v_r * (G * grad_D[0] + H * T[0])
            Vy = self.v_r * (G * grad_D[1] + H * T[1])
            Vz = self.v_r * (G * grad_D[2] + H * T[2])


            # print (self.flag_follow_obstacle)
            if(self.flag_follow_obstacle):
                # print(self.closest)

                Do = math.sqrt(self.closest[0]**2 + self.closest[1]**2 + self.closest[2]**2)

                closest_hat = [self.closest[0]/(Do+1e-8), self.closest[1]/(Do+1e-8), self.closest[2]/(Do+1e-8)]

                if (Do < self.D_hist):
                    self.D_hist = Do
                print (Do, self.D_hist)


                if(Do<self.switch_dist and (self.closest[0]*Vx+self.closest[1]*Vy+self.closest[2]*Vz)<0):
                    D_vec2 = [self.closest[0] - closest_hat[0]*self.epsilon, self.closest[1] - closest_hat[1]*self.epsilon, self.closest[2] - closest_hat[2]*self.epsilon]
                    D2 = math.sqrt(D_vec2[0]**2 + D_vec2[1]**2 + D_vec2[2]**2)
                    grad_D2 = [D_vec2[0]/D2, D_vec2[1]/D2, D_vec2[2]/D2]

                    G2 = -(2 / math.pi) * math.atan(self.k_f * D2)  # convergence
                    H2 = math.sqrt(1 - G2 ** 2)  # circulation


                    # alpha = 1.0-Do/self.switch_dist #used for a smooth transition
                    alpha = 1
                    print(alpha)
                    T_dot_gad_D2 = T[0]*grad_D2[0] + T[1]*grad_D2[1] + T[2]*grad_D2[2]
                    T2 = [T[0] - alpha*T_dot_gad_D2*grad_D2[0], T[1] - alpha*T_dot_gad_D2*grad_D2[1], T[2] - alpha*T_dot_gad_D2*grad_D2[2]]
                    norm_T2 = math.sqrt(T2[0]**2 + T2[1]**2 + T2[2]**2)
                    T2 = [T2[0]/norm_T2, T2[1]/norm_T2, T2[2]/norm_T2]

                    # print(T2)
                    # print(T2[0]*grad_D2[0] + T2[1]*grad_D2[1] + T2[2]*grad_D2[2])

                    Vx = self.v_r * (G2 * grad_D2[0] + H2 * T2[0])
                    Vy = self.v_r * (G2 * grad_D2[1] + H2 * T2[1])
                    Vz = self.v_r * (G2 * grad_D2[2] + H2 * T2[2])

                    # print("Close to obstacle")




            # Stop the robot if the it reached the end of a open path
            if not self.closed_path_flag:
                if k_min == size_traj - 1:
                    rospy.logwarn("CHECK THIS: k_min:%s size_traj-1:%s self.pos:%s local_traj[k_min]:%s", 
                        k_min, size_traj - 1, self.pos, local_traj[k_min])

                    Vx = 0
                    Vy = 0
                    Vz = 0
                    reached_endpoint = True
                    # self.reset()

            return Vx, Vy, Vz, reached_endpoint




    @staticmethod
    def insert_points(original_traj, qty_to_insert, closed_path_flag):
        """Insert points in the received path
        :param original_traj: original trajectory
        :param qty_to_insert: number of points to insert between two pair of points
        :param closed_path_flag: boolean to define if its going to be
                                 insertion of points between last and first
        :return: a new trajectory with the interpolated paths
        """
        new_traj = []
        traj_size = len(original_traj)

        if closed_path_flag:
            # Insert points between last and first
            for i in range(traj_size):
                new_traj.append(original_traj[i])

                iM = (i + 1) % traj_size
                for j in range(1, qty_to_insert + 1):
                    alpha = j / (qty_to_insert + 1.0)
                    px = (1 - alpha) * original_traj[i][0] + alpha * original_traj[iM][0]
                    py = (1 - alpha) * original_traj[i][1] + alpha * original_traj[iM][1]
                    pz = (1 - alpha) * original_traj[i][2] + alpha * original_traj[iM][2]
                    new_traj.append((px, py, pz))

        else:
            # Do not insert points between last and first
            for i in range(traj_size - 1):
                new_traj.append(original_traj[i])
                iM = i + 1
                for j in range(1, qty_to_insert + 1):
                    alpha = j / (qty_to_insert + 1.0)
                    px = (1 - alpha) * original_traj[i][0] + alpha * original_traj[iM][0]
                    py = (1 - alpha) * original_traj[i][1] + alpha * original_traj[iM][1]
                    pz = (1 - alpha) * original_traj[i][2] + alpha * original_traj[iM][2]
                    new_traj.append((px, py, pz))

        return new_traj



    @staticmethod
    def filter_path(original_traj, filter_path_n_average, closed_path_flag):
        """Filter the path using an average filter
        :param original_traj: original trajectory
        :param filter_path_n_average:
        :param closed_path_flag: boolean to define if its going to be
                                 insertion of points between last and first
        :return: a filtered list of points
        """
        size_original_traj = len(original_traj)

        if filter_path_n_average > size_original_traj:
            rospy.logwarn("Parameter 'filter_path_n_average' seems to be too high! (%d)", filter_path_n_average)

        # Force the a odd number, for symmetry
        if filter_path_n_average % 2 == 0:
            filter_path_n_average = filter_path_n_average + 1
        half = int((filter_path_n_average - 1.0) / 2.0)

        # Compute a list of shifts to further obtain the neighbor points
        ids = []
        for i in range(filter_path_n_average):
            ids.append(i - half)

        # Initialize a new list with zeros
        new_traj = []
        for i in range(size_original_traj):
            new_traj.append((0.0, 0.0, 0.0))

        # For each point in the path compute the average of the point and its neighbors
        if closed_path_flag:
            # Consider a "circular" filter
            for i in range(size_original_traj):
                for j in ids:
                    k = (i + j) % size_original_traj
                    px = new_traj[i][0] + original_traj[k][0] * float(1.0 / filter_path_n_average)
                    py = new_traj[i][1] + original_traj[k][1] * float(1.0 / filter_path_n_average)
                    pz = new_traj[i][2] + original_traj[k][2] * float(1.0 / filter_path_n_average)
                    new_traj[i] = (px, py, pz)
        else:
            # Consider a standard filter
            for i in range(size_original_traj):
                count = 0
                for j in ids:
                    k = (i + j)
                    # Decrease the number of elements in the extremities
                    if 0 <= k < size_original_traj:
                        count = count + 1
                        px = new_traj[i][0] + original_traj[k][0]
                        py = new_traj[i][1] + original_traj[k][1]
                        pz = new_traj[i][2] + original_traj[k][2]
                        new_traj[i] = (px, py, pz)

                avg_px = new_traj[i][0] / float(count)
                avg_py = new_traj[i][1] / float(count)
                avg_pz = new_traj[i][2] / float(count)
                new_traj[i] = (avg_px, avg_py, avg_pz)

        return new_traj