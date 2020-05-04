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

import uzh_fpv.flags as flags

FLAGS = flags.FLAGS
filenames = []


def gtCallback(sequence_i):
    if FLAGS.bag:
        filenames.append(flags.sequenceSensorString() + '_with_gt.bag')
    if FLAGS.zip:
        filenames.append(flags.sequenceSensorString() + '_with_gt.zip')


def noGtCallback(sequence_i):
    if FLAGS.bag:
        filenames.append(flags.sequenceSensorString() + '.bag')
    if FLAGS.zip:
        filenames.append(flags.sequenceSensorString() + '.zip')


if __name__ == '__main__':
    sys.argv = flags.FLAGS(sys.argv)
    flags.setFlagsToEachSequenceSensorAndCallCallBack(gtCallback, for_without_gt=False)
    flags.setFlagsToEachSequenceSensorAndCallCallBack(noGtCallback, for_with_gt=False)
    filenames = sorted(filenames)
    for fn in filenames:
        print(fn)
