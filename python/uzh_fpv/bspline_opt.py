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
import numpy as np

import bspline
import casadi_pose
import pose


def interpolationWeights(u):
    B = np.array([[6, 0, 0, 0],
                  [5, 3, -3, 1],
                  [1, 3, 3, -2],
                  [0, 0, 0, 1]]) * 1. / 6.
    # Not sure if reshape is necessary.
    uvec = casadi.vertcat(1, u, u**2, u**3)
    return casadi.mtimes(B, uvec)


class SymbolicTrajectory(object):
    def __init__(self, traj, is_var=True):
        """ Converts an existing trajectory into a symbolic trajectory that can be used to express casadi symbolic
         terms. Node poses are linearized around their current values. """
        self.linearized_traj = traj
        self.node_poses = []

        if is_var:
            # Optimization variable x: One col of a 6-coefficient twist for every node pose:
            # (Casadi is col-major)
            self.xvec = casadi.MX.sym('x', len(traj.node_poses) * 6, 1)
            self.x = casadi.reshape(self.xvec, 6, len(traj.node_poses))
            # Generate symbols for node poses:
            for i in range(len(traj.node_poses)):
                self.node_poses.append(casadi_pose.linearizedPose(traj.node_poses[i], self.x[:, i]))
        else:
            for i in range(len(traj.node_poses)):
                self.node_poses.append(casadi_pose.constant(traj.node_poses[i]))

        # Generate symbols for increments between poses:
        self.node_increments = []
        for i in range(len(traj.node_poses) - 1):
            self.node_increments.append((self.node_poses[i].inverse() * self.node_poses[i + 1]).asTwist())

    def _u(self, t, index):
        return (t - self.linearized_traj.node_times[index]) / self.linearized_traj.dt

    def indexOf(self, t):
        index = np.searchsorted(self.linearized_traj.node_times, t, 'right') - 1
        if 1 <= index < len(self.node_poses) - 2:
            return index
        return None

    def _sampleInterval(self, t, index):
        assert 1 <= index < len(self.node_poses) - 2
        # Equation 5 in Elias' paper:
        node_increments = [self.node_increments[index + j - 2] for j in [1, 2, 3]]
        bs = interpolationWeights(self._u(t, index))[1:]
        # Equation 4:
        pincr = [casadi_pose.fromTwist(bs[i] * node_increments[i]) for i in range(3)]
        return self.node_poses[index - 1] * pincr[0] * pincr[1] * pincr[2]

    def sample(self, t, tvar=None):
        """ If tvar is not None, it is used as time variable. It should have a value close to t. """
        index = self.indexOf(t)
        if index is None:
            return None
        return self._sampleInterval(t if tvar is None else tvar, index)

    def newTrajectory(self, solution_x):
        twists = np.array(solution_x).reshape((len(self.linearized_traj.node_poses), 6))
        new_node_poses = [self.linearized_traj.node_poses[i] * pose.fromTwist(twists[i])
                          for i in range(len(self.linearized_traj.node_poses))]
        for p in new_node_poses:
            p.fix()
        return bspline.Trajectory(self.linearized_traj.dt, self.linearized_traj.node_times, new_node_poses)


class NumericWrapper(object):
    def __init__(self, sym_traj, x_value=None):
        self.sym_traj = sym_traj
        if x_value is None:
            self.x_value = np.zeros(sym_traj.x.shape)
        else:
            self.x_value = x_value

    def sample(self, t):
        f = casadi.Function('f', [self.sym_traj.x], [self.sym_traj.sample(t).asMX()])
        Rt = np.array(f.call([self.x_value])[0])
        return pose.Pose(Rt[:, :3], Rt[:, 3].reshape((3, 1)))
