#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from enum import Enum
from itertools import groupby
from math import pi, sqrt, cos, sin, tan, atan

global exp_as_func
global u_trick
# u_trick = [0]

class distancefield_class():


    # def __init__(self, v_r, k_f, epsilon, switch_dist, is_forward_motion_flag, flag_follow_obstacle):
    def __init__(self, v_r, k_f, reverse_direction, flag_follow_obstacle, epsilon, switch_dist_0, switch_dist):

        global u_trick
        global exp_as_func

        # base variables
        self.pos = [0, 0, 0]
        #self.rpy = [0, 0, 0]
        #self.quat = [1, 0, 0, 0]
        self.path = []
        self.state_k = 0
        self.state_k_delta = 10

        self.flag_from_equation = False
        self.equation_str = "[0, 0, 0]"
        self.equation_samples = []
        self.u_vec = []
        self.state_u = 0
        self.state_u_delta = 0.02
        self.u_i = 0.0
        self.u_f = 1.0
        self.exp_as_func = None
        self.u_trick = [0]

        exp_as_func = False
        u_trick = [0]

        #Obstacle follower parameters
        self.epsilon = epsilon
        self.switch_dist_0 = switch_dist_0
        self.switch_dist = switch_dist
        self.closest_world = [0,0,0]
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



    def set_pos(self, pos):
        self.pos = pos
        # self.rpy = rpy
        # self.quat = quat


    def set_closest(self, point):
        self.closest_world = point
        # self.rpy = rpy
        # self.quat = quat


    def is_ready(self):


        if (self.flag_from_equation):
            return exp_as_func
        else:
            return True if self.path and len(self.path) > 0 else False


    def set_path(self, path, insert_n_points, filter_path_n_average, closed_path_flag):
        """Callback to obtain the path to be followed by the robot
        :param data: path ROS message
        """
        # self.reset()

        self.flag_from_equation = False

        # remove consecutive points and check for repeated points at the start and tail
        path = [x[0] for x in groupby(path)]
        if len(path) > 1:
            if path[0] == path[-1]:
                path.pop(-1)

        self.path = path
        self.closed_path_flag = closed_path_flag

        # Insert points on the path
        if insert_n_points > 0:
            self.path = self.insert_points(self.path, insert_n_points, closed_path_flag)

        # Filter the points (average filter)
        if filter_path_n_average > 0:
            self.path = self.filter_path(self.path, filter_path_n_average, closed_path_flag)

        # Update the closest index - index to the closest point in the curve
        self.state_k = 0
        d = float("inf")
        for k in range(len(self.path)):
            d_temp = math.sqrt((self.pos[0] - self.path[k][0]) ** 2 + (self.pos[1] - self.path[k][1]) ** 2 + (self.pos[2] - self.path[k][2]) ** 2)
            if d_temp < d:
                self.state_k = k
                d = d_temp

        rospy.loginfo("New path received by the controller (%d points)", len(self.path))



    def set_equation(self, equation_str, u_i, u_f, closed_path_flag, N):
        """Callback to obtain the path to be followed by the robot
        :param data: path ROS message
        """
        # self.reset()

        self.equation_str = equation_str
        self.closed_path_flag = closed_path_flag
        self.flag_from_equation = True

        self.u_i = u_i
        self.u_f = u_f

        if (self.u_i > self.u_f):
            print ("\33[41mError: u_i > u_f\33[0m")

        self.state_u_delta = (self.u_f-self.u_i)/50.0


        du = self.u_f/N
        u = self.u_i-du


        # Loop to sample the curve
        self.equation_samples = [[],[],[]]
        self.u_vec = []
        for k in range(N):

            # Increment parameter
            u = u + du

            self.u_vec.append(u)

            point = eval(self.equation_str)

            # Save the computed point
            self.equation_samples[0].append(point[0])
            self.equation_samples[1].append(point[1])
            self.equation_samples[2].append(point[2])

        # Update the closest index - index to the closest point in the curve
        self.state_k = 0
        d = float("inf")
        for k in range(len(self.equation_samples)):
            d_temp = math.sqrt((self.pos[0] - self.equation_samples[k][0]) ** 2 + (self.pos[1] - self.equation_samples[k][1]) ** 2 + (self.pos[2] - self.equation_samples[k][2]) ** 2)
            if d_temp < d:
                self.state_k = k
                d = d_temp
        self.state_u = self.u_vec[self.state_k]



        eq_aux = self.equation_str.replace("u","u_trick[0]")
        # print("\33[96m" + eq_aux + "\33[0m")
        
        global exp_as_func
        global u_trick
        u_trick = [0]
        # self.exp_as_func = eval('lambda: ' + self.equation_str)
        exp_as_func = eval('lambda: ' + eq_aux)

        rospy.loginfo("New path received by the controller (equation)")




    def sample_curve(self, u):

        global exp_as_func
        global u_trick

        # r = eval(self.equation_str) #slow?

        u_trick[0] = u
        r = exp_as_func() #fast

        # print (self.exp_as_func)

        return r



    # @staticmethod
    # def get_norm(self, arr):


    #     n = 0
    #     for k in range(len(arr)):
    #         n = n + arr[k]**2

    #     n = sqrt(n)

    #     return n



    def golden_search(self,pos,al,bl):

        x = pos[0]
        y = pos[1]
        z = pos[2]

        GOLD = 0.6180339887


        s_a = bl - GOLD*(bl-al);
        s_b = al + GOLD*(bl-al);

        ra = self.sample_curve(s_a);
        dist_vec = [x-ra[0], y-ra[1], z-ra[2]];
        # func_a = sqrt(dist_vec[0]**2 + dist_vec[1]**2 + dist_vec[2]**2)
        func_a = self.get_norm(dist_vec)

        rb = self.sample_curve(s_b);
        dist_vec = [x-rb[0], y-rb[1], z-rb[2]];
        # func_b = sqrt(dist_vec[0]**2 + dist_vec[1]**2 + dist_vec[2]**2)
        func_b = self.get_norm(dist_vec)


        k=0;
        while(bl-al > 0.00001):

            k = k+1;
            if (k==50):
                break


            if(func_a > func_b):
                al = s_a
                s_a = s_b
                s_b = al + GOLD*(bl-al)
            else:
                bl = s_b
                s_b = s_a
                s_a = bl - GOLD*(bl-al)

            ra = self.sample_curve(s_a)
            dist_vec = [x-ra[0], y-ra[1], z-ra[2]]
            func_a = self.get_norm(dist_vec)

            rb = self.sample_curve(s_b)
            dist_vec = [x-rb[0], y-rb[1], z-rb[2]]
            func_b = self.get_norm(dist_vec)
    
        u_star = (al+bl)/2;
        # print (k)

        return u_star






    # def get_GH_follower(self, delta):

    #     # Gain functions
    #     G_ = -(2 / math.pi) * math.atan(self.k_f * delta)  # convergence
    #     H_ = math.sqrt(1 - G_ ** 2)  # circulation
    #     return G_, H_







    def field_from_equation(self, pos):

        # print("\33[95mfield_from_equation\33[0m")

        x = pos[0]
        y = pos[1]
        z = pos[2]

        reached_endpoint = False


        u_star = self.golden_search(pos,self.state_u-self.state_u_delta,self.state_u+self.state_u_delta)
        self.state_u = u_star
        # print(self.state_u)
        if(self.state_u>self.u_f):
            self.state_u = self.state_u-(self.u_f-self.u_i)
        elif(self.state_u<self.u_i):
            self.state_u = self.state_u+(self.u_f-self.u_i)

        point_star = self.sample_curve(u_star)

        D_vec = [x - point_star[0], y - point_star[1], z - point_star[2]]
        D = self.get_norm(D_vec)
        # compute the gradient of the distance Function
        grad_D = [D_vec[0] / (D + 0.000001), D_vec[1] / (D + 0.000001), D_vec[2] / (D + 0.000001)]



        
        point_M = self.sample_curve(u_star+0.0001)
        point_m = self.sample_curve(u_star-0.0001)


        # numerically compute the tangent vector and normalize it
        T = [point_M[0] - point_m[0], point_M[1] - point_m[1], point_M[2] - point_m[2]]
        norm_T = self.get_norm(T)
        # math.sqrt(T[0] ** 2 + T[1] ** 2 + T[2] ** 2) + 0.000001

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
            # print(self.closest_world)

            closest_vec = [self.closest_world[0]-self.pos[0], self.closest_world[1]-self.pos[1], self.closest_world[2]-self.pos[2]]

            Do = math.sqrt(closest_vec[0]**2 + closest_vec[1]**2 + closest_vec[2]**2)

            closest_hat = [closest_vec[0]/(Do+1e-8), closest_vec[1]/(Do+1e-8), closest_vec[2]/(Do+1e-8)]

            if (Do < self.D_hist):
                self.D_hist = Do
            # print (Do, self.D_hist)



            if(Do<self.switch_dist_0 and (closest_vec[0]*Vx+closest_vec[1]*Vy+closest_vec[2]*Vz)>0):
                D_vec2 = [-(closest_vec[0] - closest_hat[0]*self.epsilon), -(closest_vec[1] - closest_hat[1]*self.epsilon), -(closest_vec[2] - closest_hat[2]*self.epsilon)]
                D2 = math.sqrt(D_vec2[0]**2 + D_vec2[1]**2 + D_vec2[2]**2)
                grad_D2 = [D_vec2[0]/D2, D_vec2[1]/D2, D_vec2[2]/D2]

                G2 = -(2 / math.pi) * math.atan(self.k_f * D2)  # convergence
                H2 = math.sqrt(1 - G2 ** 2)  # circulation

                # alpha = 1.0-Do/self.switch_dist #used for a smooth transition
                # alpha = 1
                # T_dot_gad_D2 = T[0]*grad_D2[0] + T[1]*grad_D2[1] + T[2]*grad_D2[2]
                # T2 = [T[0] - alpha*T_dot_gad_D2*grad_D2[0], T[1] - alpha*T_dot_gad_D2*grad_D2[1], T[2] - alpha*T_dot_gad_D2*grad_D2[2]]
                # norm_T2 = math.sqrt(T2[0]**2 + T2[1]**2 + T2[2]**2)
                # T2 = [T2[0]/norm_T2, T2[1]/norm_T2, T2[2]/norm_T2]


                alpha = 1
                V = [Vx/self.v_r, Vy/self.v_r, Vz/self.v_r]
                V_dot_gad_D2 = V[0]*grad_D2[0] + V[1]*grad_D2[1] + V[2]*grad_D2[2]
                T2 = [V[0] - alpha*V_dot_gad_D2*grad_D2[0], V[1] - alpha*V_dot_gad_D2*grad_D2[1], V[2] - alpha*V_dot_gad_D2*grad_D2[2]]
                norm_T2 = math.sqrt(T2[0]**2 + T2[1]**2 + T2[2]**2)
                T2 = [T2[0]/norm_T2, T2[1]/norm_T2, T2[2]/norm_T2]

                # print(T2)
                # print(T2[0]*grad_D2[0] + T2[1]*grad_D2[1] + T2[2]*grad_D2[2])

                Vx_o = self.v_r * (G2 * grad_D2[0] + H2 * T2[0])
                Vy_o = self.v_r * (G2 * grad_D2[1] + H2 * T2[1])
                Vz_o = self.v_r * (G2 * grad_D2[2] + H2 * T2[2])
                # Vx = Vx_o
                # Vy = Vy_o
                # Vz = Vz_o

                # print "V_dot_gad_D2", V_dot_gad_D2
                # print "T", T
                # print "T2", T2

                # a = 1/0

                # print "A"
                if(Do<self.switch_dist):
                    Vx = Vx_o
                    Vy = Vy_o
                    Vz = Vz_o
                    # print "B"
                else:
                    # print "C"
                    theta = (Do-self.switch_dist)/(self.switch_dist_0-self.switch_dist)
                    # print theta
                    Vx = theta*Vx + (1-theta)*Vx_o
                    Vy = theta*Vy + (1-theta)*Vy_o
                    Vz = theta*Vz + (1-theta)*Vz_o
                    norma = sqrt(Vx**2 + Vy**2 + Vz**2)
                    Vx = self.v_r*Vx/norma
                    Vy = self.v_r*Vy/norma
                    Vz = self.v_r*Vz/norma


        return Vx, Vy, Vz, reached_endpoint






    def field_from_points(self, pos):

        x, y, z = pos
        local_path = self.path
        size_path = len(local_path)
        reached_endpoint = False

        # Compute the closest ponit on the curve
        # Consider only the points in the vicinity of the current closest point (robustness)
        k_vec = [self.state_k - self.state_k_delta + i for i in range(self.state_k_delta)]
        k_vec.append(self.state_k)
        k_vec = k_vec + [self.state_k + 1 + i for i in range(self.state_k_delta)]
        for k in range(len(k_vec)):
            if k_vec[k] < 0:
                k_vec[k] = k_vec[k] + size_path
            if k_vec[k] >= size_path:
                k_vec[k] = k_vec[k] - size_path


        # iterate over the k_vec indices to get the closest point
        D = float("inf")
        k_min = size_path
        for k in k_vec:
            D_temp = math.sqrt((x - local_path[k][0]) ** 2 + (y - local_path[k][1]) ** 2 + (z - local_path[k][2]) ** 2)
            if D_temp < D:
                k_min = k
                D = D_temp
        self.state_k = k_min  # index of the closest point

        # print ("self.state_k: ", self.state_k)

        # compute the distance vector
        D_vec = [x - local_path[k_min][0], y - local_path[k_min][1], z - local_path[k_min][2]]
        # print("self.pos: ", self.pos)
        # print("x, y, z: ", x, y, z)
        # print("D_vec: ", D_vec)
        # print("local_path[k][2]: ", local_path[k][2])
        # print("")
        # compute the gradient of the distance Function
        grad_D = [D_vec[0] / (D + 0.000001), D_vec[1] / (D + 0.000001), D_vec[2] / (D + 0.000001)]

        # compute neighbors of k_min
        k1 = k_min - 1  # previous index
        k2 = k_min + 1  # next index
        if self.closed_path_flag:
            # consider that the first point and the last are neighbors
            if k1 == -1:
                k1 = size_path - 1
            if k2 == size_path:
                k2 = 0
        else:
            # consider that the first point and the last are distant apart
            if k1 == -1:
                k1 = 0
            if k2 == size_path:
                k2 = size_path - 1

        # numerically compute the tangent vector and normalize it
        T = [local_path[k2][0] - local_path[k1][0], local_path[k2][1] - local_path[k1][1], local_path[k2][2] - local_path[k1][2]]
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
            # print(self.closest_world)

            closest_vec = [self.closest_world[0]-self.pos[0], self.closest_world[1]-self.pos[1], self.closest_world[2]-self.pos[2]]

            Do = math.sqrt(closest_vec[0]**2 + closest_vec[1]**2 + closest_vec[2]**2)

            closest_hat = [closest_vec[0]/(Do+1e-8), closest_vec[1]/(Do+1e-8), closest_vec[2]/(Do+1e-8)]

            if (Do < self.D_hist):
                self.D_hist = Do
            # print (Do, self.D_hist)

            # print "D"
            if(Do<self.switch_dist_0 and (closest_vec[0]*Vx+closest_vec[1]*Vy+closest_vec[2]*Vz)>0):
                D_vec2 = [-(closest_vec[0] - closest_hat[0]*self.epsilon), -(closest_vec[1] - closest_hat[1]*self.epsilon), -(closest_vec[2] - closest_hat[2]*self.epsilon)]
                D2 = math.sqrt(D_vec2[0]**2 + D_vec2[1]**2 + D_vec2[2]**2)
                grad_D2 = [D_vec2[0]/D2, D_vec2[1]/D2, D_vec2[2]/D2]

                G2 = -(2 / math.pi) * math.atan(self.k_f * D2)  # convergence
                H2 = math.sqrt(1 - G2 ** 2)  # circulation


                # # alpha = 1.0-Do/self.switch_dist #used for a smooth transition
                # alpha = 1
                # # print(alpha)
                # T_dot_gad_D2 = T[0]*grad_D2[0] + T[1]*grad_D2[1] + T[2]*grad_D2[2]
                # T2 = [T[0] - alpha*T_dot_gad_D2*grad_D2[0], T[1] - alpha*T_dot_gad_D2*grad_D2[1], T[2] - alpha*T_dot_gad_D2*grad_D2[2]]
                # norm_T2 = math.sqrt(T2[0]**2 + T2[1]**2 + T2[2]**2)
                # T2 = [T2[0]/norm_T2, T2[1]/norm_T2, T2[2]/norm_T2]

                alpha = 1
                V = [Vx/self.v_r, Vy/self.v_r, Vz/self.v_r]
                V_dot_gad_D2 = V[0]*grad_D2[0] + V[1]*grad_D2[1] + V[2]*grad_D2[2]
                T2 = [V[0] - alpha*V_dot_gad_D2*grad_D2[0], V[1] - alpha*V_dot_gad_D2*grad_D2[1], V[2] - alpha*V_dot_gad_D2*grad_D2[2]]
                norm_T2 = math.sqrt(T2[0]**2 + T2[1]**2 + T2[2]**2)
                T2 = [T2[0]/norm_T2, T2[1]/norm_T2, T2[2]/norm_T2]

                # print(T2)
                # print(T2[0]*grad_D2[0] + T2[1]*grad_D2[1] + T2[2]*grad_D2[2])

                Vx_o = self.v_r * (G2 * grad_D2[0] + H2 * T2[0])
                Vy_o = self.v_r * (G2 * grad_D2[1] + H2 * T2[1])
                Vz_o = self.v_r * (G2 * grad_D2[2] + H2 * T2[2])

                # print "C"
                if(Do<self.switch_dist):
                    Vx = Vx_o
                    Vy = Vy_o
                    Vz = Vz_o
                    # print "A"
                else:
                    # print "B"
                    theta = (Do-self.switch_dist)/(self.switch_dist_0-self.switch_dist)
                    Vx = theta*Vx + (1-theta)*Vx_o
                    Vy = theta*Vy + (1-theta)*Vy_o
                    Vz = theta*Vz + (1-theta)*Vz_o
                    norma = sqrt(Vx**2 + Vy**2 + Vz**2)
                    Vx = self.v_r*Vx/norma
                    Vy = self.v_r*Vy/norma
                    Vz = self.v_r*Vz/norma


        # Stop the robot if the it reached the end of a open path
        if not self.closed_path_flag:
            if k_min == size_path - 1:
                rospy.logwarn("CHECK THIS: k_min:%s size_path-1:%s self.pos:%s local_path[k_min]:%s", 
                    k_min, size_path - 1, self.pos, local_path[k_min])

                Vx = 0
                Vy = 0
                Vz = 0
                reached_endpoint = True
                # self.reset()

        return Vx, Vy, Vz, reached_endpoint







    def compute_field_at_p(self, pos):
        """Compute the vector field that will guide the robot through a path
        :return:
            Vx, Vy, reached_endpoint
        """

        if (self.flag_from_equation):
            Vx, Vy, Vz, reached_endpoint = self.field_from_equation(pos)
        else:
            Vx, Vy, Vz, reached_endpoint = self.field_from_points(pos)
        

        return Vx, Vy, Vz, reached_endpoint





    def vec_field_path(self):


        Vx, Vy, Vz, reached_endpoint = self.compute_field_at_p(self.pos)

        return Vx, Vy, Vz, reached_endpoint







    @staticmethod
    def get_norm(arr):

        n = 0
        for k in range(len(arr)):
            n = n + arr[k]**2

        n = sqrt(n)

        return n



    @staticmethod
    def insert_points(original_path, qty_to_insert, closed_path_flag):
        """Insert points in the received path
        :param original_path: original path
        :param qty_to_insert: number of points to insert between two pair of points
        :param closed_path_flag: boolean to define if its going to be
                                 insertion of points between last and first
        :return: a new path with the interpolated paths
        """
        new_path = []
        path_size = len(original_path)

        if closed_path_flag:
            # Insert points between last and first
            for i in range(path_size):
                new_path.append(original_path[i])

                iM = (i + 1) % path_size
                for j in range(1, qty_to_insert + 1):
                    alpha = j / (qty_to_insert + 1.0)
                    px = (1 - alpha) * original_path[i][0] + alpha * original_path[iM][0]
                    py = (1 - alpha) * original_path[i][1] + alpha * original_path[iM][1]
                    pz = (1 - alpha) * original_path[i][2] + alpha * original_path[iM][2]
                    new_path.append((px, py, pz))

        else:
            # Do not insert points between last and first
            for i in range(path_size - 1):
                new_path.append(original_path[i])
                iM = i + 1
                for j in range(1, qty_to_insert + 1):
                    alpha = j / (qty_to_insert + 1.0)
                    px = (1 - alpha) * original_path[i][0] + alpha * original_path[iM][0]
                    py = (1 - alpha) * original_path[i][1] + alpha * original_path[iM][1]
                    pz = (1 - alpha) * original_path[i][2] + alpha * original_path[iM][2]
                    new_path.append((px, py, pz))

        return new_path



    @staticmethod
    def filter_path(original_path, filter_path_n_average, closed_path_flag):
        """Filter the path using an average filter
        :param original_path: original path
        :param filter_path_n_average:
        :param closed_path_flag: boolean to define if its going to be
                                 insertion of points between last and first
        :return: a filtered list of points
        """
        size_original_path = len(original_path)

        if filter_path_n_average > size_original_path:
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
        new_path = []
        for i in range(size_original_path):
            new_path.append((0.0, 0.0, 0.0))

        # For each point in the path compute the average of the point and its neighbors
        if closed_path_flag:
            # Consider a "circular" filter
            for i in range(size_original_path):
                for j in ids:
                    k = (i + j) % size_original_path
                    px = new_path[i][0] + original_path[k][0] * float(1.0 / filter_path_n_average)
                    py = new_path[i][1] + original_path[k][1] * float(1.0 / filter_path_n_average)
                    pz = new_path[i][2] + original_path[k][2] * float(1.0 / filter_path_n_average)
                    new_path[i] = (px, py, pz)
        else:
            # Consider a standard filter
            for i in range(size_original_path):
                count = 0
                for j in ids:
                    k = (i + j)
                    # Decrease the number of elements in the extremities
                    if 0 <= k < size_original_path:
                        count = count + 1
                        px = new_path[i][0] + original_path[k][0]
                        py = new_path[i][1] + original_path[k][1]
                        pz = new_path[i][2] + original_path[k][2]
                        new_path[i] = (px, py, pz)

                avg_px = new_path[i][0] / float(count)
                avg_py = new_path[i][1] / float(count)
                avg_pz = new_path[i][2] / float(count)
                new_path[i] = (avg_px, avg_py, avg_pz)

        return new_path