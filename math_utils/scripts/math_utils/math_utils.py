#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import *

# --------------------------
# ---------- test ----------
# --------------------------

def test():
	return 1


# -----------------------------
# ---------- Generic ----------
# -----------------------------

def get_norm(arr):
	#Description:
	#Get the Euclidean norm of an array

	n = 0
	for k in range(len(arr)):
		n = n + arr[k]**2
	n = sqrt(n)

	return n


def normalize(list_in):
	#Description:
	#Normalize a vector to the unit norm

	n = get_norm(list_in)
	list_out = [i/n for i in list_in]

	return list_out


# ---------------------------------
# ---------- Quaternions ----------
# ---------------------------------

def quat_conj(q):
	#Description:
	#Conjugate of a quaternion

	return [q[0], -q[1], -q[2], -q[3]]


def quat_mult(q1,q2):
	#Description:
	#Quaternion multiplication q1 x q2
	
	a1 = q1[0]
	b1 = q1[1]
	c1 = q1[2]
	d1 = q1[3]
	a2 = q2[0]
	b2 = q2[1]
	c2 = q2[2]
	d2 = q2[3]

	q = [a1*a2 - b1*b2 - c1*c2 - d1*d2, 
		 a1*b2 + b1*a2 + c1*d2 - d1*c2,
		 a1*c2 - b1*d2 + c1*a2 + d1*b2,
		 a1*d2 + b1*c2 - c1*b2 + d1*a2]

	return q


def quat_apply_rot(q,u):
	#Description:
	#Apply the rotation given by q to the vector u

	v = quat_mult(quat_mult(q,[0]+u),quat_conj(q))

	return [v[1], v[2], v[3]]


def qpos(q):
	#Description:
	#Force a quaternion to have a positve real part
	
	if (q[0]<0):
		q = [-i for i in q]

	return q


def d_quat_from_omega(q,w):
	#Description:
	#Compute the qaternion derivative given a angulr sepeed w
	#ngular speed w is on th world/body???????? frame

	wx = w[0]
	wy = w[1]
	wz = w[2]
	return [0.5*( 0*q[0] - wx*q[1] - wy*q[2] - wz*q[3]),
			0.5*(wx*q[0] +  0*q[1] + wz*q[2] - wy*q[3]),
			0.5*(wy*q[0] - wz*q[1] +  0*q[2] + wx*q[3]),
			0.5*(wz*q[0] + wy*q[1] - wx*q[2] +  0*q[3]) ]


# ------------------------------
# ---------- Matrices ----------
# ------------------------------

def mat_times_v(M,v):
	#Description:
	#Matrix times vector multiplication
	
	if (not len(M[0]) == len(v)):
		print ("\33[91m[ERROR] mat_times_v, M and v sizes are invalid for multiplication\33[0m")
		return False

	out = []

	for l in range(len(M)):
		out.append(0)
		for c in range(len(M[l])):
			out[-1] = out[-1] + M[l][c]*v[c]

	return out


def mat_times_mat(M1,M2):
	#Description:
	#Matrix times matrix multiplication

	L1 = len(M1)
	C1 = len(M1[0])
	L2 = len(M2)
	C2 = len(M2[0])

	if (not C1 == L2):
		print ("\33[91m[ERROR] mat_times_mat, M1 and M2 sizes are invalid for multiplication\33[0m")
		return False

	M = [[0 for j in range(C2)] for i in range(L1)]

	for i in range(L1):
		for j in range(C2):
			for k in range(C1):
				M[i][j] = M[i][j] + M1[i][k]*M2[k][j]

	return M


def mat_plus_mat(M1,M2):
	#Description:
	#Matrix plus matrix addition

	L1 = len(M1)
	C1 = len(M1[0])
	L2 = len(M2)
	C2 = len(M2[0])

	if (not (L1 == L2 and C1 == C2)):
		print ("\33[91m[ERROR] mat_times_mat, M1 and M2 sizes are invalid for addition\33[0m")
		return False

	M = [[0 for j in range(C1)] for i in range(L1)]

	for i in range(L1):
		for j in range(C2):
				M[i][j] = M1[i][j] + M2[i][j]

	return M


def mat_minus_mat(M1,M2):
	#Description:
	#Matrix plus matrix subtraction

	L1 = len(M1)
	C1 = len(M1[0])
	L2 = len(M2)
	C2 = len(M2[0])

	if (not (L1 == L2 and C1 == C2)):
		print ("\33[91m[ERROR] mat_minus_mat, M1 and M2 sizes are invalid for subtraction\33[0m")
		return

	M = [[0 for j in range(C1)] for i in range(L1)]

	for i in range(L1):
		for j in range(C2):
				M[i][j] = M1[i][j] - M2[i][j]

	return M


def vec_plus_vec(v1,v2):
	#Description:
	#Vetor plus vector addition

	L1 = len(v1)
	L2 = len(v2)

	if (not (L1 == L2)):
		print ("\33[91m[ERROR] vec_plus_vec, v1 and v2 sizes are invalid for addition\33[0m")
		return

	v = [0 for i in range(L1)]

	for i in range(L1):
		v[i] = v1[i] + v2[i]

	return v


def vec_minus_vec(v1,v2):
	#Description:
	#Vetor minus vector subtraction

	L1 = len(v1)
	L2 = len(v2)

	if (not (L1 == L2)):
		print ("\33[91m[ERROR] vec_minus_vec, v1 and v2 sizes are invalid for subtraction\33[0m")
		return

	v = [0 for i in range(L1)]

	for i in range(L1):
		v[i] = v1[i] - v2[i]

	return v


def mat_transpose(Min):
	#Description:
	#Matrix transpose

	L = len(Min)
	C = len(Min[0])

	Mout = [[0 for j in range(L)] for i in range(C)]


	for i in range(L):
		for j in range(C):
			Mout[j][i] = Min[i][j]

	return Mout


def mat_inv(M):
	#Description:
	#Matrix inverse

	M = np.matrix(M)
	M = np.linalg.inv(M)

	return M.tolist()


def mat_identity(n):
	#Description:
	#Returns identity matrix of n dimensions

	I = [[0 for j in range(n)] for i in range(n)]
	for k in range(n):
		I[k][k] = 1.0

	return I


def mat_zeros(n):
	#Description:
	#Returns zero matrix of n dimensions

	M = [[0 for j in range(n)] for i in range(n)]

	return M


# ---------------------------------
# ---------- Conversions ----------
# --------------------------------- 

def eul2quat(a):
	#Description:
	#Converts Euler angles to quaternion
	#Euler: ZYX on local frame (Roll, Pitch, Yaw) in radians
	#Quaternion: qw, qx, qy, qz

	q = [0,0,0,0]
	phi = a[0]/2
	theta = a[1]/2
	psi = a[2]/2
	q[0] = cos(phi)*cos(theta)*cos(psi) + sin(phi)*sin(theta)*sin(psi)
	q[1] = sin(phi)*cos(theta)*cos(psi) - cos(phi)*sin(theta)*sin(psi)
	q[2] = cos(phi)*sin(theta)*cos(psi) + sin(phi)*cos(theta)*sin(psi)
	q[3] = cos(phi)*cos(theta)*sin(psi) - sin(phi)*sin(theta)*cos(psi)

	return q
""" 
#Function to convert euler angles to a quaternion
def eul2quat(ang_in):
	#Description:
	#Converts Euler angles to quaternion
	#Euler: ZYX on local frame (Roll, Pitch, Yaw) in radians
	#Quaternion: qw, qx, qy, qz

	cr = cos(ang_in[0] * 0.5); #roll
	sr = sin(ang_in[0] * 0.5); #roll
	cp = cos(ang_in[1] * 0.5); #pitch
	sp = sin(ang_in[1] * 0.5); #pitch
	cy = cos(ang_in[2] * 0.5); #yaw
	sy = sin(ang_in[2] * 0.5); #yaw

	q_out = [1,0,0,0]
	q_out[0] = cy * cp * cr + sy * sp * sr
	q_out[1] = cy * cp * sr - sy * sp * cr
	q_out[2] = sy * cp * sr + cy * sp * cr
	q_out[3] = sy * cp * cr - cy * sp * sr

	return q_out
 """


def euler2rotm(self, rpy):
	#Description:
	#Converts Euler angles to rotation matrix
	#Euler: ZYX on local frame (Roll, Pitch, Yaw) in radians

	phi = rpy[0]
	theta = rpy[1]
	psi = rpy[2]
	# Get rotation matrix
	Rot << [[(cos(theta)*cos(psi)), (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)), (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))],
			[(cos(theta)*sin(psi)), (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)), (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))],
			[(-sin(theta)), (sin(phi)*cos(theta)), (cos(phi)*cos(theta))]]

	return Rot


def quat2rotm(q):
	#Description:
	#Converts quaternion to rotation matrix
	#Quaternion: qw, qx, qy, qz

	qw = q[0]
	qx = q[1]
	qy = q[2]
	qz = q[3]

	Rot = [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
		[2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
		[2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]]; #this was checked on matlab
			
	return Rot
	

def quat2euler(q):
	#Description:
	#Converts quaternion to Euler angles
	#Quaternion: qw, qx, qy, qz
	#Euler: ZYX on local frame (Roll, Pitch, Yaw) in radians

	angle = [0.0, 0.0, 0.0]

	# roll (x-axis rotation)
	sinr_cosp = +2.0 * (q[0] * q[1] + q[2] * q[3])
	cosr_cosp = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
	angle[0] = atan2(sinr_cosp, cosr_cosp)

	# pitch (y-axis rotation)
	sinp = +2.0 * (q[0] * q[2] - q[3] * q[1])
	if (fabs(sinp) >= 1):
		angle[1] = copysign(pi / 2, sinp) # use 90 degrees if out of range
	else:
		angle[1] = asin(sinp)

	# yaw (z-axis rotation)
	siny_cosp = +2.0 * (q[0] * q[3] + q[1] * q[2])
	cosy_cosp = +1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
	angle[2] = atan2(siny_cosp, cosy_cosp)

	return angle


def rotm2quat(R):
	#Description:
	#Converts rotation matrix to quaternion
	#Quaternion: qw, qx, qy, qz

	tr = R[0][0]+R[1][1]+R[2][2]

	if (tr > 0):
		S = sqrt(tr+1.0) * 2 # S=4*qw 
		qw = 0.25 * S
		qx = (R[2][1] - R[1][2]) / S
		qy = (R[0][2] - R[2][0]) / S
		qz = (R[1][0] - R[0][1]) / S
	elif ((R[0][0] > R[1][1]) and (R[0][0] > R[2][2])):
		S = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2 # S=4*qx 
		qw = (R[2][1] - R[1][2]) / S
		qx = 0.25 * S
		qy = (R[0][1] + R[1][0]) / S
		qz = (R[0][2] + R[2][0]) / S
	elif (R[1][1] > R[2][2]):
		S = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2 # S=4*qy
		qw = (R[0][2] - R[2][0]) / S
		qx = (R[0][1] + R[1][0]) / S
		qy = 0.25 * S;
		qz = (R[1][2] + R[2][1]) / S
	else:
		S = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2 # S=4*qz
		qw = (R[1][0] - R[0][1]) / S
		qx = (R[0][2] + R[2][0]) / S
		qy = (R[1][2] + R[2][1]) / S
		qz = 0.25 * S

	n = sqrt(qw**2+qx**2+qy**2+qz**2)
	if(qw>0):
		q = [qw/n,qx/n,qy/n,qz/n]
	else:
		q = [-qw/n,-qx/n,-qy/n,-qz/n]

	return q


def quat2axang(q):
	#Description:
	#Converts quaternion to axis/angle representation
	#Quaternion: qw, qx, qy, qz
	#Axis/Angle: angle in radians and axis has unit norm

	if(q[0]<0):
		q = [-q[0], -q[1], -q[2], -q[3]]

	s = sqrt(q[1]**2+q[2]**2+q[3]**2) + 0.000001
	axis = [q[1]/s, q[2]/s, q[3]/s]
	axis = normalize(axis)

	ang = 2 * acos(q[0])

	return axis, ang


def rotm2axang(R):
	#Description:
	#Converts rotation matrix to axis/angle representation
	#Axis/Angle: angle in radians and axis has unit norm

	q = rotm2quat(R)

	axis, ang = quat2axang(q)

	return axis, ang

