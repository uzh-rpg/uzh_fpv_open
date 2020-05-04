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

import casadi
import faulthandler
from IPython.core import ultratb
import IPython
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

from align_spline_to_leica import SplineLeicaAlignment
import uzh_fpv.casadi_pose as casadi_pose
import uzh_fpv.flags as flags
from uzh_fpv.generalized_time import GeneralizedTime
import uzh_fpv.leica as leica
import uzh_fpv.linear_interpolation as linear_interpolation
import uzh_fpv.min_bags as min_bags
import uzh_fpv.plots as plots
import uzh_fpv.pose as pose

FLAGS = flags.FLAGS


class GroundTruthInLeicaFrame(object):
    def __init__(self, ground_truth, alignment):
        self.T_G_I_of_t = ground_truth
        self.alignment = alignment

    def times(self):
        return [i - GeneralizedTime(float(self.alignment.t_G_L)) for i in self.T_G_I_of_t.times]

    def prismPositions(self):
        p_L_Ps = np.array([(self.alignment.T_L_G * T_G_I * self.alignment.p_I_P).ravel()
                          for T_G_I in self.T_G_I_of_t.poses])
        return p_L_Ps

    def imuPoses(self):
        return [self.alignment.T_L_G * T_G_I for T_G_I in self.T_G_I_of_t.poses]

    def imuPositions(self):
        p_L_Is = np.array([(self.alignment.T_L_G * T_G_I).t.ravel()
                          for T_G_I in self.T_G_I_of_t.poses])
        return p_L_Is

    def getLinearInterpolation(self):
        t_L_PGTs = self.times()
        t_G0_PGTs = [(i - t_L_PGTs[0]).to_sec() for i in t_L_PGTs]
        return linear_interpolation.Trajectory(t_G0_PGTs, self.imuPoses())


def numericalErrors(T_L_I_of_t_G0, t_G0_Ps, p_L_Ps, p_I_P):
    errors = []
    for t_G0_P, p_L_P in zip(t_G0_Ps, p_L_Ps):
        if T_L_I_of_t_G0.indexOf(t_G0_P) is None:
            continue
        T_L_I = T_L_I_of_t_G0.sample(t_G0_P)
        p_L_P_guess = (T_L_I * p_I_P).ravel()
        errors.append(np.linalg.norm(p_L_P - p_L_P_guess))
    print('Have %d errors' % len(errors))
    return errors


def symbolicalErrors(T_L_I_of_t_G0, t_G0_Ps, p_L_Ps, sym_p_I_P, sym_t_corr_G0, sym_T_corr_L_twist):
    errvec = casadi.MX.sym('', 0, 0)
    T_corr_L = casadi_pose.fromTwist(sym_T_corr_L_twist)
    for t_G0_P, p_L_P in zip(t_G0_Ps, p_L_Ps):
        T_L_I = T_L_I_of_t_G0.timeDerivativeSample(t_G0_P + sym_t_corr_G0, t_G0_P)
        if T_L_I is None:
            continue
        p_corr_P_guess = (T_corr_L * T_L_I * sym_p_I_P)
        sqn = casadi_pose.squareNorm(p_L_P - p_corr_P_guess)
        # Robust loss: Flatten the loss of errors > 5cm, most likely outliers.
        # TODO completely discard a portion of worst measurements instead?
        rloss = casadi.atan(sqn / 0.0025)
        errvec = casadi.vertcat(errvec, rloss)
    return errvec


def run():
    alignment = SplineLeicaAlignment()
    alignment.deserialize()
    ground_truth = min_bags.readGroundTruthFromImuGtoBag()
    t_L_Ps, p_L_Ps = leica.parseTextFile(os.path.join(flags.rawDataPath(), 'leica.txt'))

    while True:
        aligned_gt = GroundTruthInLeicaFrame(ground_truth, alignment)

        t_L_PGTs = aligned_gt.times()
        t_G0_PGTs = [(i - t_L_PGTs[0]).to_sec() for i in t_L_PGTs]
        t_G0_Ps = [(i - t_L_PGTs[0]).to_sec() for i in t_L_Ps]

        # Calculate errors
        T_L_I_of_t_G0 = aligned_gt.getLinearInterpolation()
        num_errors = numericalErrors(T_L_I_of_t_G0, t_G0_Ps, p_L_Ps, aligned_gt.alignment.p_I_P)
        print('Mean square error is %f' % np.mean(np.square(num_errors)))

        # Empirically, optimization only made stuff worse after this (?)
        # break

        opti = casadi.Opti()
        t_corr_G0 = opti.variable()
        T_corr_L_twist = opti.variable(6)
        p_I_P = opti.variable(3)
        opti.set_initial(p_I_P, aligned_gt.alignment.p_I_P)

        errvec = symbolicalErrors(T_L_I_of_t_G0, t_G0_Ps, p_L_Ps, p_I_P, t_corr_G0, T_corr_L_twist)
        opti.minimize(casadi.sum1(errvec))
        opti.solver('ipopt')

        sol = opti.solve()
        upd_coeffs = np.hstack((sol.value(t_corr_G0), sol.value(T_corr_L_twist), sol.value(p_I_P) - alignment.p_I_P.ravel()))
        if np.max(np.abs(upd_coeffs)) < 1e-5:
            break

        alignment.t_G_L = alignment.t_G_L + sol.value(t_corr_G0)
        alignment.T_L_G = pose.fromTwist(sol.value(T_corr_L_twist)) * alignment.T_L_G
        alignment.p_I_P = sol.value(p_I_P).reshape((3, 1))

    alignment.serialize('gt_leica_alignment')
    out_files = flags.SplineLeicaAlignmentFiles('gt_leica_alignment')
    np.savetxt(out_files.errors, np.array(num_errors))

    if FLAGS.gui:
        plt.figure(0)
        plots.xyPlot(aligned_gt.imuPositions(), aligned_gt.prismPositions(), p_L_Ps)
        plt.title('x,y trajectory of %s' % flags.sequenceSensorString())

        plt.figure(1)
        plots.xyztPlot(t_G0_PGTs, aligned_gt.imuPositions(), aligned_gt.prismPositions(), t_G0_Ps, p_L_Ps)
        plt.title('x,y,z trajectory of %s' % flags.sequenceSensorString())

        plt.figure(2)
        plt.semilogx(sorted(num_errors), np.arange(len(num_errors), dtype=float) / len(num_errors))
        plt.grid()
        plt.xlabel('Leica error [m]')
        plt.ylabel('Fraction of measurements with smaller error')
        plt.title('Cumulative distribution of Leica error for %s' % flags.sequenceSensorString())
        plt.show()

    # IPython.embed()


if __name__ == '__main__':
    faulthandler.enable()
    sys.excepthook = ultratb.FormattedTB(
        mode='Plain', color_scheme='Linux', call_pdb=1)
    sys.argv = flags.FLAGS(sys.argv)
    run()
