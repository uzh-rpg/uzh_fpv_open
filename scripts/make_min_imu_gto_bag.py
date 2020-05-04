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
import sys

import rosbag

import uzh_fpv.flags as flags


def makeMinImuGtoBag():
    in_bag = rosbag.Bag(flags.groundTruthBagPath())
    out_bag = rosbag.Bag(flags.minImuGroundTruthOdometryBagPath(), 'w')
    for bag_msg in in_bag:
        msg = bag_msg.message
        if bag_msg.topic == flags.imuTopic():
            out_bag.write(flags.imuTopic(), msg, msg.header.stamp)
        if bag_msg.topic == '/groundtruth/odometry':
            out_bag.write('/groundtruth/odometry', msg, msg.header.stamp)

    out_bag.flush()
    out_bag.close()


if __name__ == '__main__':
    faulthandler.enable()
    sys.excepthook = ultratb.FormattedTB(
        mode='Plain', color_scheme='Linux', call_pdb=1)
    sys.argv = flags.FLAGS(sys.argv)

    makeMinImuGtoBag()
