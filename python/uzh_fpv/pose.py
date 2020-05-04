# Copyright (C) 2020 Titus Cieslewski, RPG, University of Zurich, Switzerland
#   You can contact the author at <titus at ifi dot uzh dot ch>
# Copyright (C) 2020 Davide Scaramuzza, RPG, University of Zurich, Switzerland
#
# This file is part of uzh_fpv_open.
#
# uzh_fpv_open is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# uzh_fpv_open is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with uzh_fpv_open. If not, see <http:#www.gnu.org/licenses/>.

import numpy as np
import pyquaternion
import scipy.linalg


def cross2Matrix(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def matrix2Cross(M):
    skew = (M - M.T)/2
    return np.array([-skew[1, 2], skew[0, 2], -skew[0, 1]])


class Pose(object):
    def __init__(self, R, t):
        assert type(R) is np.ndarray
        assert type(t) is np.ndarray
        assert R.shape == (3, 3)
        assert t.shape == (3, 1)
        self.R = R
        self.t = t

    def inverse(self):
        return Pose(self.R.T, -np.dot(self.R.T, self.t))

    def __mul__(self, other):
        if isinstance(other, Pose):
            return Pose(np.dot(self.R, other.R), np.dot(self.R, other.t) + self.t)
        if type(other) is np.ndarray:
            assert len(other.shape) == 2
            assert other.shape[0] == 3
            return np.dot(self.R, other) + self.t
        raise Exception('Multiplication with unknown type!')

    def asArray(self):
        return np.vstack((np.hstack((self.R, self.t)), np.array([0, 0, 0, 1])))

    def asTwist(self):
        so_matrix = scipy.linalg.logm(self.R)
        if np.sum(np.imag(so_matrix)) > 1e-10:
            raise Exception('logm called for a matrix with angle Pi. ' +
                'Not defined! Consider using another representation!')
        so_matrix = np.real(so_matrix)
        return np.hstack((np.ravel(self.t), matrix2Cross(so_matrix)))

    def q_wxyz(self):
        return pyquaternion.Quaternion(matrix=self.R).unit.q

    def fix(self):
        self.R = fixRotationMatrix(self.R)

    def __repr__(self):
        return self.asArray().__repr__()


def fromTwist(twist):
    # Using Rodrigues' formula
    w = twist[3:]
    theta = np.linalg.norm(w)
    if theta < 1e-6:
        return Pose(np.eye(3), twist[:3].reshape(3, 1))
    M = cross2Matrix(w/theta)
    R = np.eye(3) + M * np.sin(theta) + np.dot(M, M) * (1 - np.cos(theta))
    return Pose(R, twist[:3].reshape((3, 1)))


def fromPositionAndQuaternion(xyz, q_wxyz):
    R = pyquaternion.Quaternion(
        q_wxyz[0], q_wxyz[1], q_wxyz[2], q_wxyz[3]).rotation_matrix
    t = xyz.reshape(3, 1)
    return Pose(R, t)


# ROS geometry_msgs/Pose
def fromPoseMessage(pose_msg):
    pos = pose_msg.position
    ori = pose_msg.orientation
    R = pyquaternion.Quaternion(ori.w, ori.x, ori.y, ori.z).rotation_matrix
    t = np.array([pos.x, pos.y, pos.z]).reshape(3, 1)
    return Pose(R, t)


def identity():
    return Pose(np.eye(3), np.zeros((3, 1)))


def geodesicDistanceSO3(R1, R2):
    return getRotationAngle(np.dot(R1, R2.T))


def getRotationAngle(R):
    return np.arccos((np.trace(R) - 1 - 1e-6) / 2)


def fixRotationMatrix(R):
    u, _, vt = np.linalg.svd(R)
    R_new = np.dot(u, vt)
    if np.linalg.det(R_new) < 0:
        R_new = -R_new
    return R_new
