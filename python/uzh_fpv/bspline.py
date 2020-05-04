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


# Euation 6 in Elias' paper:
def interpolationWeights(u):
    B = np.array([[6, 0, 0, 0],
                  [5, 3, -3, 1],
                  [1, 3, 3, -2],
                  [0, 0, 0, 1]]) * 1. / 6.
    # Not sure if reshape is necessary.
    uvec = np.array([u ** i for i in range(4)]).reshape((4, 1))
    return np.dot(B, uvec)


def calculateIncrements(poses):
    result = []
    for i in range(len(poses) - 1):
        result.append((poses[i].inverse() * poses[i+1]).asTwist())
    return result


class Trajectory(object):
    def __init__(self, dt, node_times, node_poses):
        assert len(node_times) == len(node_poses)
        assert np.max(np.abs((node_times[1:] - node_times[:-1]) - dt)) < 1e-6

        self.dt = dt
        self.node_times = node_times
        self.node_poses = node_poses
        self.node_increments = calculateIncrements(node_poses)

    def _u(self, t, index):
        return (t - self.node_times[index]) / self.dt

    def _sampleInterval(self, t, index):
        assert 1 <= index < len(self.node_poses) - 2
        # Equation 5 in Elias' paper:
        # Fun fact: Precalculating the twists and caching them saved 90% of runtime during a specific profiling.
        node_increments = [self.node_increments[index + j - 2] for j in [1, 2, 3]]
        bs = interpolationWeights(self._u(t, index))[1:].tolist()
        # Equation 4:
        pincr = [pose.fromTwist(b * incr) for b, incr in zip(bs, node_increments)]
        return self.node_poses[index - 1] * pincr[0] * pincr[1] * pincr[2]

    def sample(self, t):
        index = np.searchsorted(self.node_times, t, 'right') - 1
        if index < 1 or index >= len(self.node_poses) - 2:
            return None
        return self._sampleInterval(t, index)

    def sampleOrdered(self, times):
        assert np.all((times[1:] - times[:-1]) > 0)
        t_i = 0
        nt_i = 1
        result = []
        while True:
            if nt_i == 1:
                while times[t_i] < self.node_times[nt_i]:
                    result.append(None)
                    t_i = t_i + 1
            while self.node_times[nt_i] <= times[t_i] < self.node_times[nt_i + 1]:
                result.append(self._sampleInterval(times[t_i], nt_i))
                t_i = t_i + 1
                if t_i == len(times):
                    break
            if t_i == len(times):
                break
            nt_i = nt_i + 1
            if nt_i == len(self.node_times) - 2:
                break
        assert len(result) <= len(times)
        while len(result) <= len(times):
            result.append(None)
        return result

    def serialize(self, path):
        np_lines = []
        for time, pose in zip(self.node_times, self.node_poses):
            np_lines.append(np.hstack((time, pose.t.ravel(), pose.q_wxyz())))
        np.savetxt(path, np.array(np_lines), header='timestamp tx ty tz qw qx qy qz')


def trajectoryFromSamples(times, poses, dt):
    assert np.all((times[1:] - times[:-1]) > 0)
    control_times = np.arange(times[0], times[-1], dt)
    control_poses = []
    ct_i = 0
    for t_i in range(len(times) - 1):
        if times[t_i] <= control_times[ct_i] < times[t_i + 1]:
            increment = (poses[t_i].inverse() * poses[t_i + 1]).asTwist()
            factor = (control_times[ct_i] - times[t_i]) / dt
            control_poses.append(poses[t_i] * pose.fromTwist(factor * increment))
            ct_i = ct_i + 1
            if ct_i == len(control_times):
                break
    assert len(control_times) == len(control_poses)
    return Trajectory(dt, control_times, control_poses)


def trajectoryFromFile(path):
    times = []
    poses = []
    serialized = np.loadtxt(path)
    for line in serialized:
        times.append(line[0])
        poses.append(pose.fromPositionAndQuaternion(line[1:4], line[4:]))
    dt = times[1] - times[0]
    return Trajectory(dt, np.array(times), poses)
