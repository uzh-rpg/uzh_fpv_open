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

import IPython
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

import rosbag

import uzh_fpv.flags as flags
from uzh_fpv.generalized_time import GeneralizedTime
import uzh_fpv.leica as leica

FLAGS = flags.FLAGS


if __name__ == '__main__':
    sys.argv = flags.FLAGS(sys.argv)

    t, positions = leica.parseTextFile(os.path.join(flags.rawDataPath(), 'leica.txt'))

    if not os.path.exists(flags.minImuGroundTruthOdometryBagPath()):
        raise Exception('Run make_min_imu_gto_bag first!')
    bag = rosbag.Bag(flags.minImuGroundTruthOdometryBagPath())

    gt_times = []
    gt_positions = []

    for bag_msg in bag:
        msg = bag_msg.message
        if bag_msg.topic == '/groundtruth/odometry':
            gt_times.append(GeneralizedTime(msg.header.stamp))
            gt_positions.append(np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z]))

    gt_positions = np.array(gt_positions)

    plt.plot([(i - t[0]).to_sec() for i in t], positions[:, 0], '.', label='leica')
    plt.plot([(i - t[0]).to_sec() for i in gt_times], gt_positions[:, 0], label='ground truth')
    plt.grid()
    plt.title('x(t) versus Leica measurements of %s' % flags.sequenceString())
    plt.xlabel('t')
    plt.ylabel('x')
    plt.legend()
    plt.show()
