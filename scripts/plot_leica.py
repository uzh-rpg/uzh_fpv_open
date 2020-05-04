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

import uzh_fpv.flags as flags
import uzh_fpv.leica as leica


if __name__ == '__main__':
    sys.argv = flags.FLAGS(sys.argv)

    if not os.path.exists(flags.minImuGroundTruthOdometryBagPath()):
        raise Exception('Run make_min_imu_gto_bag first!')
    bag = rosbag.Bag(flags.minImuGroundTruthOdometryBagPath())

    gt_times = []
    gt_positions = []

    for bag_msg in bag:
        msg = bag_msg.message
        if bag_msg.topic == '/groundtruth/odometry':
            gt_times.append(msg.header.stamp)
            gt_positions.append(np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z]))

    gt_positions = np.array(gt_positions)

    t, positions = leica.parseTextFile(os.path.join(flags.rawDataPath(), 'leica.txt'))

    plt.plot(positions[:, 0], positions[:, 1], '.', label='leica')
    plt.plot(gt_positions[:, 0], gt_positions[:, 1], label='ground truth')
    plt.grid()
    plt.legend()
    plt.title('x,y trajectory of %s' % flags.sequenceString())
    plt.gca().set_aspect('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()
