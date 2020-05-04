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

import sys
from zipfile import ZipFile

import rosbag

import uzh_fpv.flags as flags

FLAGS = flags.FLAGS


def deGroundTruthBag():
    in_bag = rosbag.Bag(flags.groundTruthBagPath())
    out_bag = rosbag.Bag(flags.noGroundTruthBagPath(), 'w')
    for bag_msg in in_bag:
        msg = bag_msg.message
        if 'groundtruth' in bag_msg.topic:
            continue
        out_bag.write(bag_msg.topic, msg, msg.header.stamp)
    out_bag.flush()
    out_bag.close()


def deGroundTruthZip():
    in_zip = ZipFile(flags.groundTruthZipPath(), 'r')
    out_zip = ZipFile(flags.noGroundTruthZipPath(), 'w')
    for item in in_zip.infolist():
        if item.filename == 'groundtruth.txt':
            continue
        out_zip.writestr(item, in_zip.read(item.filename))
    out_zip.close()


def callback(sequence_i):
    if sequence_i % FLAGS.num_workers == FLAGS.worker_i:
        if FLAGS.bag:
            print('De-ground truthing %s bag...' % flags.sequenceSensorString())
            deGroundTruthBag()
        if FLAGS.zip:
            print('De-ground truthing %s zip...' % flags.sequenceSensorString())
            deGroundTruthZip()


if __name__ == '__main__':
    sys.argv = flags.FLAGS(sys.argv)
    flags.setFlagsToEachSequenceSensorAndCallCallBack(callback, for_with_gt=False)
