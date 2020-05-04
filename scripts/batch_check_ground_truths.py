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

import os
from StringIO import StringIO
import sys
from zipfile import ZipFile

import rosbag

import uzh_fpv.flags as flags

from compare_gyro import compareGyro
from make_min_imu_gto_bag import makeMinImuGtoBag

FLAGS = flags.FLAGS


def checkZipIsSameAsBag():
    assert os.path.exists(flags.groundTruthZipPath())
    assert os.path.exists(flags.groundTruthBagPath())
    z = ZipFile(flags.groundTruthZipPath(), 'r')
    bag = rosbag.Bag(flags.groundTruthBagPath())
    bag_info = bag.get_type_and_topic_info()

    # Same ground truth length?
    zip_gt = StringIO(z.read('groundtruth.txt'))
    zip_num_gt = len([i for i in zip_gt]) - 1
    assert bag_info.topics['/groundtruth/odometry'].message_count == zip_num_gt
    assert bag_info.topics['/groundtruth/pose'].message_count == zip_num_gt

    # Same IMU length?
    zip_num_imu = len([i for i in StringIO(z.read('imu.txt'))]) - 1
    assert bag_info.topics[flags.imuTopic()].message_count == zip_num_imu

    # TODO sensor-specific checks...


def callback(sequence_i):
    if sequence_i % FLAGS.num_workers == FLAGS.worker_i:
        if FLAGS.bag:
            if FLAGS.redo_min_bags or not os.path.exists(flags.minImuGroundTruthOdometryBagPath()):
                print('Generating min bag for %s' % flags.sequenceSensorString())
                makeMinImuGtoBag()
            print('Comparing bag gyro for %s' % flags.sequenceSensorString())
            compareGyro()
        if FLAGS.zip:
            print('Checking zip looks same as bag for %s' % flags.sequenceSensorString())
            checkZipIsSameAsBag()


if __name__ == '__main__':
    sys.argv = flags.FLAGS(sys.argv)
    flags.setFlagsToEachSequenceSensorAndCallCallBack(callback)
