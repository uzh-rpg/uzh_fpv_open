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

import faulthandler
from IPython.core import ultratb
import matplotlib.pyplot as plt
import numpy as np
import os
import pyquaternion
import sys

import rosbag

import uzh_fpv.flags as flags

FLAGS = flags.FLAGS


def compareGyro():
    gyro_stamps = []
    gyros = []

    gt_attitude_stamps = []
    gt_attitudes = []

    if not os.path.exists(flags.minImuGroundTruthOdometryBagPath()):
        raise Exception('Run make_min_imu_gto_bag first!')
    bag = rosbag.Bag(flags.minImuGroundTruthOdometryBagPath())

    for bag_msg in bag:
        msg = bag_msg.message
        if bag_msg.topic == flags.imuTopic():
            gyro_stamps.append(msg.header.stamp)
            gyros.append(np.array([msg.angular_velocity.x,
                                   msg.angular_velocity.y,
                                   msg.angular_velocity.z]))
        if bag_msg.topic == '/groundtruth/odometry':
            gt_attitude_stamps.append(msg.header.stamp)
            gt_attitudes.append(pyquaternion.Quaternion(
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z))

    gyro_times = [(i - gyro_stamps[0]).to_sec() for i in gyro_stamps]
    gt_times = [(i - gyro_stamps[0]).to_sec() for i in gt_attitude_stamps]

    if FLAGS.gt_from_txt != '':
        print('Loading custom ground truth...')
        loaded = np.loadtxt(FLAGS.gt_from_txt)
        gt_attitude_stamps = None
        gt_times = loaded[:, 0] - gyro_stamps[0].to_sec()
        gt_attitudes = [pyquaternion.Quaternion(i[7], i[4], i[5], i[6]) for i in loaded]

    gt_Rs = [i.rotation_matrix for i in gt_attitudes]
    gt_twists = []
    for i in range(1, len(gt_attitudes) - 1):
        relative = np.dot(gt_Rs[i - 1].T, gt_Rs[i + 1])
        gt_twists.append(np.array([
            (relative[2, 1] - relative[1, 2]) / 2,
            (relative[0, 2] - relative[2, 0]) / 2,
            (relative[1, 0] - relative[0, 1]) / 2
        ]) / (gt_times[i + 1] - gt_times[i - 1]))

    plt.plot(gyro_times, [i[0] for i in gyros], '.')
    plt.plot(gt_times[1:-1], [i[0] for i in gt_twists])
    plt.grid()
    plt.title('x gyro to gt comparison %s' % flags.sequenceSensorString())
    plt.xlabel('Time[s]')
    plt.ylabel('Angular rate [rad]')
    plt.show()


if __name__ == '__main__':
    faulthandler.enable()
    sys.excepthook = ultratb.FormattedTB(
        mode='Plain', color_scheme='Linux', call_pdb=1)
    sys.argv = FLAGS(sys.argv)

    compareGyro()
