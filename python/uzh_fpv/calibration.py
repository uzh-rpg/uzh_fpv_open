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
import pose


# https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras/include/aslam/cameras/implementation/EquidistantDistortion.hpp
class EquidistantDistortion(object):
    def __init__(self, k):
        self.k = k

    def distort(self, p):
        r = np.sqrt(p[0] * p[0] + p[1] * p[1])
        theta = np.arctan(r)
        theta2468 = np.zeros(4)
        theta2468[0] = theta * theta
        theta2468[1] = theta2468[0] * theta2468[0]
        theta2468[2] = theta2468[1] * theta2468[0]
        theta2468[3] = theta2468[1] * theta2468[1]
        thetad = theta * (1 + np.dot(self.k, theta2468))
        if r > 1e-8:
            return p * thetad / r
        else:
            return p


class CamCalibration(object):
    def __init__(self, f, c, distortion, shape, T_C_B):
        self.f = f
        self.c = c
        self.distortion = distortion
        self.shape = shape
        self.T_C_B = T_C_B
        self.T_B_C = T_C_B.inverse()

    def project(self, p_C):
        if len(p_C.shape) > 1:
            assert len(p_C.shape) == 2
            prj_rc = np.fliplr(np.array([self.project(i) for i in p_C if i[2] > 0]))
            return np.array([i for i in prj_rc if 0 <= i[0] < self.shape[0] and
                             0 <= i[1] < self.shape[1]])
        assert len(p_C) == 3
        in_plane = p_C[:2] / p_C[2]
        dist = self.distortion.distort(in_plane)
        return self.f * dist + self.c

