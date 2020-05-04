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

import casadi
import math
import numpy as np

import pose


class Pose(object):
    def __init__(self, R, t):
        assert type(R) == casadi.casadi.MX or type(R) == casadi.casadi.DM or type(R) == np.ndarray
        assert type(t) == casadi.casadi.MX or type(t) == casadi.casadi.DM or type(t) == np.ndarray
        assert R.shape == (3, 3)
        self.R = R
        if t.shape in [(1, 3), (3,)]:
            self.t = casadi.reshape(t, 3, 1)
        elif t.shape == (3, 1):
            self.t = t
        else:
            raise Exception('Invalid t dim %s' % t.shape.__repr__())

    def inverse(self):
        return Pose(self.R.T, -(casadi.mtimes(self.R.T, self.t)))

    def __mul__(self, other):
        if isinstance(other, Pose):
            return Pose(casadi.mtimes(self.R, other.R), casadi.mtimes(self.R, other.t) + self.t)
        elif isinstance(other, casadi.casadi.MX) and other.shape == (3, 1):
            return casadi.mtimes(self.R, other) + self.t
        else:
            raise NotImplementedError

    def asTwist(self):
        acosarg = (casadi.trace(self.R) - 1) / 2
        theta = casadi.acos(casadi.if_else(casadi.fabs(acosarg) >= 1., 1., acosarg))
        w = casadi.horzcat((self.R[2, 1] - self.R[1, 2]),
                           (self.R[0, 2] - self.R[2, 0]),
                           (self.R[1, 0] - self.R[0, 1]))
        return casadi.horzcat(casadi.reshape(self.t, 1, 3), theta * w / casadi.norm_2(w))

    def asMX(self):
        return casadi.horzcat(self.R, self.t)


def cross2Matrix(v):
    return casadi.vertcat(casadi.horzcat(0, -v[2], v[1]),
                          casadi.horzcat(v[2], 0, -v[0]),
                          casadi.horzcat(-v[1], v[0], 0))


def mulNumCas(num_pose, cas_pose):
    return Pose(casadi.mtimes(num_pose.R, cas_pose.R), casadi.mtimes(num_pose.R, cas_pose.t) + num_pose.t)


def squareNorm(v):
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]


def fromTwist(sym_twist):
    """ Using Rodrigues' formula """
    w = sym_twist[3:]
    theta2 = squareNorm(w)

    # Rodrigues' with Taylor expansion!
    M = cross2Matrix(w)
    M2 = casadi.mtimes(M, M)
    order = 8

    R = np.eye(3) + M + 0.5 * M2
    for i in range(1, order/2, 2):
        R = R - (theta2 ** i / math.factorial(2*i+1)) * M - (theta2 ** i / math.factorial(2*i+2)) * M2
    for i in range(2, order/2, 2):
        R = R + (theta2 ** i / math.factorial(2*i + 1)) * M + (theta2 ** i / math.factorial(2*i + 2)) * M2

    return Pose(R, sym_twist[:3])


def linearizedPose(initial_guess=None, twist=None, twist_name=None):
    if initial_guess is None:
        initial_guess = pose.identity()
    if twist is None:
        assert twist_name is not None
        twist = casadi.MX.sym(twist_name, 6, 1)
    assert type(initial_guess) == pose.Pose
    assert type(twist) == casadi.casadi.MX
    return mulNumCas(initial_guess, fromTwist(twist))


def constant(num_pose):
    return Pose(casadi.MX(num_pose.R), casadi.MX(num_pose.t))
