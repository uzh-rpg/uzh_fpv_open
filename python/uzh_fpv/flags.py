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

from absl import flags
import os


FLAGS = flags.FLAGS


# Dataset specification
flags.DEFINE_string('env', '', 'indoor / outdoor (i/o)')
flags.DEFINE_string('cam', '', 'camera (fw/45)')
flags.DEFINE_string('nr', '0', 'sequence number')

# Sensor specification
flags.DEFINE_string('sens', '', 'davis or snap')

# Ground truth source override
flags.DEFINE_string('gt_from_txt', '', 'Use text file for ground truth.')

# Batching flags
flags.DEFINE_bool('bag', True, 'Treat bags?')
flags.DEFINE_bool('zip', True, 'Treat zips?')
flags.DEFINE_bool('redo_min_bags', False, '')
flags.DEFINE_integer('num_workers', 4, '')
flags.DEFINE_integer('worker_i', 0, '')

# Optimization flags
flags.DEFINE_float('dt', .1, 'Period between nodes')
flags.DEFINE_integer('errs', 1000, 'Error sample count')

flags.DEFINE_bool('gui', True, 'Plot stuff?')


def initial_t_G_L():
    """ Good initial guesses for the time offset. """
    if FLAGS.env == 'o':
        if FLAGS.cam == 'fw' and FLAGS.nr in ['9', '10']:
            return -25265.
        if FLAGS.cam == '45':
            return 3600.
    return -1.


def envCamString():
    if FLAGS.env == 'i':
        result = 'indoor_'
    else:
        assert FLAGS.env == 'o'
        result = 'outdoor_'
    if FLAGS.cam == 'fw':
        result = result + 'forward'
    else:
        assert FLAGS.cam == '45'
        result = result + '45'
    return result


def sequenceString():
    return envCamString() + '_' + FLAGS.nr


def sensorString():
    if FLAGS.sens == 'davis':
        return 'davis'
    else:
        assert FLAGS.sens == 'snap'
        return 'snapdragon'


def sequenceSensorString():
    return sequenceString() + '_' + sensorString()


def calibString():
    result = envCamString() + '_calib_'
    if FLAGS.sens == 'davis':
        result = result + 'davis'
    else:
        assert FLAGS.sens == 'snap'
        result = result + 'snapdragon'
    return result


def repoRoot():
    return os.path.dirname(os.path.dirname(os.path.dirname(__file__)))


def groundTruthBagPath():
    return os.path.join(repoRoot(), 'output', sequenceSensorString() + '_with_gt.bag')


def noGroundTruthBagPath():
    return os.path.join(repoRoot(), 'output', sequenceSensorString() + '.bag')


def groundTruthZipPath():
    return os.path.join(repoRoot(), 'output', sequenceSensorString() + '_with_gt.zip')


def noGroundTruthZipPath():
    return os.path.join(repoRoot(), 'output', sequenceSensorString() + '.zip')


def unzippedGtPath():
    wd = os.path.join(repoRoot(), 'output')
    wgt = sequenceSensorString() + '_with_gt'
    result = os.path.join(wd, wgt)
    if not os.path.exists(result):
        print('Unzipping %s' % wgt)
        os.system('cd %s && unzip %s -d %s' % (wd, wgt + '.zip', wgt))
    return result


def calibPath():
    return os.path.join(repoRoot(), 'calib', calibString())


def minImuGroundTruthOdometryBagPath():
    min_bag_path = os.path.join(repoRoot(), 'min_bags')
    if not os.path.exists(min_bag_path):
        os.makedirs(min_bag_path)
    return os.path.join(min_bag_path, sequenceSensorString() + '_imu_gto.bag')


class GtFittedSplineFiles(object):
    def __init__(self):
        self.root = os.path.join(repoRoot(), 'intermediate', 'gt_fitted_spline', sequenceSensorString())
        self.spline = os.path.join(self.root, 'spline.txt')
        self.t_GT_spline = os.path.join(self.root, 't_GT_spline.txt')


class SplineLeicaAlignmentFiles(object):
    def __init__(self, name='spline_leica_alignment'):
        self.root = os.path.join(repoRoot(), 'intermediate', name, sequenceSensorString())
        self.t_G_L = os.path.join(self.root, 't_G_L.txt')
        self.T_L_G = os.path.join(self.root, 'T_L_G.txt')
        self.p_I_P = os.path.join(self.root, 'p_I_P.txt')
        self.errors = os.path.join(self.root, 'errors.txt')


def plotPath():
    path = os.path.join(repoRoot(), 'plots', sequenceSensorString())
    if not os.path.exists(path):
        os.makedirs(path)
    return path


def rawDataPath():
    return os.path.join(repoRoot(), 'raw', sequenceString())


def imuTopic():
    if FLAGS.sens == 'davis':
        return '/dvs/imu'
    else:
        assert FLAGS.sens == 'snap'
        return '/snappy_imu'


def setFlagsToEachSequenceSensorAndCallCallBack(callback, for_with_gt=True, for_without_gt=True):
    """ An index is passed to the first arugment of the callback. """
    i = 0
    FLAGS.env = 'i'
    FLAGS.cam = 'fw'
    for nr, with_gt in [(3, True), (5, True), (6, True), (7, True), (8, False), (9, True), (10, True), (11, False),
                        (12, False)]:
        if with_gt:
            if not for_with_gt:
                continue
        if not with_gt:
            if not for_without_gt:
                continue
        FLAGS.nr = str(nr)
        for sens in ['davis', 'snap']:
            FLAGS.sens = sens
            callback(i)
            i = i + 1
    FLAGS.cam = '45'
    for nr, with_gt in [(1, False), (2, True), (3, False), (4, True), (9, True), (11, False), (12, True), (13, True),
                        (14, True), (16, False)]:
        if with_gt:
            if not for_with_gt:
                continue
        if not with_gt:
            if not for_without_gt:
                continue
        FLAGS.nr = str(nr)
        for sens in ['davis', 'snap']:
            FLAGS.sens = sens
            callback(i)
            i = i + 1
    FLAGS.env = 'o'
    FLAGS.cam = 'fw'
    for nr, with_gt in [(1, True), (2, False), (3, True), (5, True), (6, False), (9, False), (10, False)]:
        if with_gt:
            if not for_with_gt:
                continue
        if not with_gt:
            if not for_without_gt:
                continue
        FLAGS.nr = str(nr)
        for sens in ['davis', 'snap']:
            FLAGS.sens = sens
            callback(i)
            i = i + 1
    FLAGS.cam = '45'
    for nr, with_gt in [(1, True), (2, False)]:
        if with_gt:
            if not for_with_gt:
                continue
        if not with_gt:
            if not for_without_gt:
                continue
        FLAGS.nr = str(nr)
        for sens in ['davis', 'snap']:
            FLAGS.sens = sens
            callback(i)
            i = i + 1
