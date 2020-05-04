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

import casadi_pose
import pose


class Trajectory(object):
    def __init__(self, times, poses):
        """Meant for high-frequency samples. Not usable for optimization. """
        self.times = times
        self.poses = poses

    def indexOf(self, t):
        index = np.searchsorted(self.times, t, 'right') - 1
        if 0 <= index < len(self.poses) - 1:
            return index
        return None

    def _sampleInterval(self, t, index):
        assert 0 <= index < len(self.poses) - 1
        increment = (self.poses[index].inverse() * self.poses[index + 1]).asTwist()
        factor = (t - self.times[index]) / (self.times[index + 1] - self.times[index])
        return self.poses[index] * pose.fromTwist(factor * increment)

    def sample(self, t):
        index = self.indexOf(t)
        if index is None:
            return None
        return self._sampleInterval(t, index)

    def _timeDerivativeSampleInterval(self, tvar, index):
        increment = (self.poses[index].inverse() * self.poses[index + 1]).asTwist()
        factor = (tvar - self.times[index]) / (self.times[index + 1] - self.times[index])
        return casadi_pose.mulNumCas(self.poses[index], casadi_pose.fromTwist(factor * increment))

    def timeDerivativeSample(self, tvar, initial_t):
        """ initial_t needed for linearization """
        index = self.indexOf(initial_t)
        if index is None:
            return None
        return self._timeDerivativeSampleInterval(tvar, index)
