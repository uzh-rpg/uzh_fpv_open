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

import matplotlib.pyplot as plt
import numpy as np
import os
import sys

import rosbag

import uzh_fpv.bspline as bspline
import uzh_fpv.flags as flags
import uzh_fpv.pose as pose

FLAGS = flags.FLAGS


def getSamplesFromBag():
    if not os.path.exists(flags.minImuGroundTruthOdometryBagPath()):
        raise Exception('Run make_min_imu_gto_bag first!')
    bag = rosbag.Bag(flags.minImuGroundTruthOdometryBagPath())

    gt_times = []
    gt_poses = []

    for bag_msg in bag:
        msg = bag_msg.message
        if bag_msg.topic == '/groundtruth/odometry':
            gt_times.append(msg.header.stamp)
            gt_poses.append(pose.fromPoseMessage(msg.pose.pose))

    first_stamp = gt_times[0]
    gt_times = np.array([(i - first_stamp).to_sec() for i in gt_times])

    return gt_times, gt_poses, first_stamp


def considerDiscretization(gt_times, gt_poses, dt):
    clean_times = []
    clean_poses = []
    for t, p in zip(gt_times, gt_poses):
        if len(clean_times) == 0 or t > clean_times[-1]:
            clean_times.append(t)
            clean_poses.append(p)
        else:
            assert t == clean_times[-1]
            assert pose.geodesicDistanceSO3(p.R, clean_poses[-1].R) < 2e-3
            assert np.linalg.norm(p.t - clean_poses[-1].t) < 1e-5
    clean_times = np.array(clean_times)

    traj = bspline.trajectoryFromSamples(clean_times, clean_poses, dt)

    test_times = clean_times
    test_poses = traj.sampleOrdered(test_times)

    valid_times = []
    valid_poses = []
    valid_mask = []
    for time, t_pose in zip(test_times, test_poses):
        valid_mask.append(t_pose is not None)
        if t_pose is not None:
            valid_times.append(time)
            valid_poses.append(t_pose)

    return traj, valid_times, valid_poses, valid_mask


if __name__ == '__main__':
    sys.argv = flags.FLAGS(sys.argv)
    # pr = cProfile.Profile()
    # pr.enable()
    gt_times, gt_poses, _ = getSamplesFromBag()

    discs = [2. ** -i for i in range(5)]
    mterrs = []
    mRerrs = []

    for d_i, disc in enumerate(discs):
        print('Considering dt=%f' % disc)

        traj, test_times, test_poses, valid_mask = considerDiscretization(gt_times, gt_poses, disc)
        # pr.dump_stats('evaluate_discretizations.profile')

        valid_gt_poses = [i for i, v in zip(gt_poses, valid_mask) if v]

        terrs = np.array([np.linalg.norm(vgp.t - tp.t) for vgp, tp in zip(valid_gt_poses, test_poses)])
        mterr = terrs.mean()
        print('Mean position error is %f m' % mterr)
        mterrs.append(mterr)

        Rerrs = np.array([np.rad2deg(pose.geodesicDistanceSO3(vgp.R, tp.R)) for vgp, tp in zip(valid_gt_poses, test_poses)])
        assert np.all(Rerrs >= 0.)
        mRerr = Rerrs.mean()
        print('Mean rotation error is %f degrees' % mRerr)
        mRerrs.append(mRerr)

        plt.figure(d_i)
        for dim, name in enumerate(['x', 'y', 'z']):
            plt.plot(gt_times, [i.t[dim] for i in gt_poses], label=name)
            plt.plot(traj.node_times, [i.t[dim] for i in traj.node_poses], 'x', label='%s nodes' % name)
            plt.plot(test_times, [i.t[dim] for i in test_poses], '.', label='%s samples' % name)
        plt.grid()
        # plt.legend()
        plt.title('Disc %.3f s, %s' % (disc, flags.sequenceSensorString()))
        plt.xlabel('t')
        plt.ylabel('position')

    plt.figure(d_i + 1)

    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel('dt = 2^-i')
    ax1.set_ylabel('position error [m]', color=color)
    ax1.semilogy(mterrs, color=color)
    ax1.grid()

    ax2 = ax1.twinx()

    color = 'tab:blue'
    ax2.set_ylabel('rotation error [degrees]', color=color)
    ax2.semilogy(mRerrs, color=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped

    plt.show()
