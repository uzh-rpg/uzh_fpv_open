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
import sys

import uzh_fpv.flags as flags

FLAGS = flags.FLAGS


errss = []
p_I_Ps = []
done = set()


def callback(sequence_i):
    if FLAGS.sens == 'davis':
        plt.figure(0)
    else:
        assert FLAGS.sens == 'snap'
        plt.figure(1)

    files = flags.SplineLeicaAlignmentFiles('gt_leica_alignment')
    errs = np.loadtxt(files.errors)

    if FLAGS.env == 'i':
        color = plt.get_cmap('tab10').colors[0]
    else:
        color = plt.get_cmap('tab10').colors[1]
    if (FLAGS.sens, FLAGS.env) not in done:
        label = 'outdoor' if FLAGS.env == 'o' else 'indoor'
        done.add((FLAGS.sens, FLAGS.env))
    else:
        label = None
    plt.semilogx(sorted(errs), np.arange(len(errs), dtype=float) / len(errs), '.', color=color, ms=0.5, label=label)

    errss.append((flags.sequenceSensorString(), errs))

    p_I_Ps.append((flags.sequenceSensorString(), np.loadtxt(files.p_I_P).ravel()))


def readableError(e):
    if e < 0.01:
        return '%.2fmm' % (e * 1000)
    if e < 0.1:
        return '%.2fcm' % (e * 100)
    if e < 1:
        return '**%.2fcm**' % (e * 100)
    return '**%.2fm**' % e


if __name__ == '__main__':
    sys.argv = flags.FLAGS(sys.argv)
    flags.setFlagsToEachSequenceSensorAndCallCallBack(callback)

    for sens in ['davis', 'snapdragon']:
        print('\n\n## Leica error table (%s):\n' % sens)

        percentiles = [50, 60, 70, 80, 90]
        header = '| Name |'
        sep = '|------|'
        for p in percentiles:
            header = header + (' %d |' % p)
            sep = sep + '----|'
        print(header)
        print(sep)
        for name, errs in errss:
            if sens in name:
                row = '| %s |' % name
                perrs = np.percentile(errs, percentiles)
                for perr in perrs:
                    row = row + (' %s |' % readableError(perr))
                print row

    for sens in ['davis', 'snapdragon']:
        for cam in ['forward', '45']:
            print('\n\n## p_I_P table (%s, %s):\n' % (sens, cam))

            print('| Name | x | y | z |')
            print('|------|---|---|---|')
            for name, p_I_P in p_I_Ps:
                if sens in name and cam in name:
                    print('|%s|%f|%f|%f|' % (name, p_I_P[0], p_I_P[1], p_I_P[2]))

    for f, s in enumerate(['DAVIS', 'Snapdragon']):
        plt.figure(f)
        plt.yticks(np.arange(0, 1.1, 0.1))
        plt.ylim([0, 1])
        plt.grid()
        plt.legend()
        plt.xlabel('Leica error [m]')
        plt.ylabel('Fraction of measurements with smaller error')
        plt.title('Cumulative distribution of Leica errors (%s)' % s)
    plt.show()
