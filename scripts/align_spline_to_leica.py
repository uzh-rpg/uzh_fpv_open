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
import copy
import faulthandler
from IPython.core import ultratb
import matplotlib
import numpy as np
import os
import sys

import uzh_fpv.bspline as bspline
import uzh_fpv.bspline_opt as bspline_opt
import uzh_fpv.casadi_pose as casadi_pose
import uzh_fpv.flags as flags
import uzh_fpv.leica as leica
import uzh_fpv.min_bags as min_bags
import uzh_fpv.pose as pose
import uzh_fpv.util as util

FLAGS = flags.FLAGS


def getTimeOfCorrespondingDavisGt():
    assert FLAGS.sens == 'snap'
    FLAGS.sens = 'davis'
    davis_gt = min_bags.readGroundTruthFromImuGtoBag()
    FLAGS.sens = 'snap'
    return davis_gt.times[0].to_sec()


class SplineLeicaAlignment(object):
    def __init__(self, t_G_G0=None):
        self.t_G_L = flags.initial_t_G_L()
        if t_G_G0 is not None:
            if t_G_G0.secs < 1500000000:
                # Indicates that something is wrong - happens with Snapdragon.
                self.t_G_L = flags.initial_t_G_L() - getTimeOfCorrespondingDavisGt() + t_G_G0.to_sec()
        self.T_L_G = pose.identity()
        self.p_I_P = np.zeros((3, 1))

    def serialize(self, name='spline_leica_alignment'):
        files = flags.SplineLeicaAlignmentFiles(name)
        if not os.path.exists(files.root):
            os.makedirs(files.root)
        np.savetxt(files.t_G_L, np.array([self.t_G_L]))
        np.savetxt(files.T_L_G, self.T_L_G.asArray())
        np.savetxt(files.p_I_P, self.p_I_P)

    def deserialize(self):
        files = flags.SplineLeicaAlignmentFiles()
        self.t_G_L = np.loadtxt(files.t_G_L)
        T_L_G_matrix = np.loadtxt(files.T_L_G)
        self.T_L_G = pose.Pose(T_L_G_matrix[:3, :3], T_L_G_matrix[:3, 3].reshape((3, 1)))
        self.p_I_P = np.loadtxt(files.p_I_P).reshape((3, 1))


class Progress(object):
    def __init__(self, T_G0_I_num, t_L_Ps, p_L_Ps, t_G_G0):
        self.t_L_Ps = t_L_Ps
        self.p_L_Ps = p_L_Ps
        self.t_G_G0 = t_G_G0
        self.alignment = SplineLeicaAlignment(t_G_G0)

        # Derived values (bcause T_L_G is identity initially)
        self.T_L_I = copy.copy(T_G0_I_num)
        self.T_G_I = T_G0_I_num

    def update(self, t_corr_G0, T_L_G_twist, p_I_P):
        self.alignment.t_G_L = self.alignment.t_G_L + float(t_corr_G0)
        print('Time offset is now %f' % self.alignment.t_G_L)
        self.alignment.T_L_G = self.alignment.T_L_G * pose.fromTwist(np.array(T_L_G_twist).ravel())
        print('T_L_G is')
        print(self.alignment.T_L_G)
        print('Has translation of %f and rotation of %f degrees' %
              (np.linalg.norm(self.alignment.T_L_G.t),
               np.rad2deg(pose.getRotationAngle(self.alignment.T_L_G.R))))
        self.T_L_I.node_poses = [self.alignment.T_L_G * i for i in self.T_G_I.node_poses]
        print('p_I_P is %s' % np.array2string(np.array(p_I_P).ravel()))
        self.alignment.p_I_P = np.array(p_I_P).reshape((3, 1))

    def t_G0_Ps(self):
        return [(i - self.t_G_G0).to_sec() + self.alignment.t_G_L for i in self.t_L_Ps]

    def plot(self, errs=None):
        time_samples = np.linspace(self.T_L_I.node_times[1] + 0.1, self.T_L_I.node_times[-3] - 0.1, 1000)

        # Plot initial situation (xy and in time):
        plt.figure(0)
        imu_positions = np.array([self.T_L_I.sample(t).t.ravel() for t in time_samples])
        prism_positions = np.array([(self.T_L_I.sample(t) * self.alignment.p_I_P).ravel() for t in time_samples])
        plots.xyPlot(imu_positions, prism_positions, self.p_L_Ps)
        plt.title('x,y trajectory of %s' % flags.sequenceSensorString())

        plt.figure(1)
        plots.xyztPlot(time_samples, imu_positions, prism_positions, self.t_G0_Ps(), self.p_L_Ps)
        plt.title('x,y,z trajectory of %s' % flags.sequenceSensorString())

        if errs is not None:
            plt.figure(2)
            plt.semilogx(sorted(errs), np.arange(len(errs), dtype=float) / len(errs))
            plt.grid()
            plt.xlabel('Leica error [m]')
            plt.ylabel('Fraction of measurements with smaller error')
            plt.title('Cumulative distribution of Leica error for %s' % flags.sequenceSensorString())

        plt.show()


def run():
    # Frames: L = Leica frame, G = ground truth frame, I = IMU frame, P = prism frame,
    # G0/L0 = same as G in space, but time centered at the first measurement
    t_L_Ps, p_L_Ps = leica.parseTextFile(os.path.join(flags.rawDataPath(), 'leica.txt'))
    spline_files = flags.GtFittedSplineFiles()
    T_G0_I_num = bspline.trajectoryFromFile(spline_files.spline)
    t_G_G0 = util.rostimeFromFile(spline_files.t_GT_spline)

    progress = Progress(T_G0_I_num, t_L_Ps, p_L_Ps, t_G_G0)
    if FLAGS.gui:
        progress.plot()

    print('Defining symbols...')
    # Optimization variables:
    t_corr_G0 = casadi.MX.sym('tcG0')  # Time offset
    T_L_G_twist = casadi.MX.sym('TLG', 6, 1)
    T_G0_I_of_t = bspline_opt.SymbolicTrajectory(T_G0_I_num, is_var=False)
    p_I_P = casadi.MX.sym('pIP', 3, 1)  # Prism position in IMU frame

    while True:
        T_L_G = casadi_pose.linearizedPose(initial_guess=progress.alignment.T_L_G, twist=T_L_G_twist)

        print('Expressing error terms...')
        errvec = casadi.MX.sym('', 0, 0)
        for t_G0_P, p_L_P in zip(progress.t_G0_Ps(), p_L_Ps):
            t_c_P = t_corr_G0 + t_G0_P
            t_c_P_guess = t_G0_P
            if T_G0_I_of_t.indexOf(t_c_P_guess) is None:
                continue
            T_G0_I = T_G0_I_of_t.sample(t_c_P_guess, tvar=t_c_P)
            p_G0_P = T_G0_I * p_I_P
            p_L_P_guess = T_L_G * p_G0_P
            errvec = casadi.vertcat(errvec, casadi_pose.squareNorm(p_L_P - p_L_P_guess))
        print('Have %d error terms' % errvec.shape[0])

        print('Optimizing...')
        x = casadi.vertcat(t_corr_G0, T_L_G_twist, p_I_P)
        nlp = {'x': x, 'f': casadi.sum1(errvec)}
        S = casadi.nlpsol('S', 'ipopt', nlp)
        x0 = np.zeros((10, 1))
        x0[-3:] = progress.alignment.p_I_P
        r = S(x0=x0)

        delta = np.array(r['x'])
        delta[-3:] = delta[-3:] - progress.alignment.p_I_P
        print('Optimization delta is:')
        print(delta.ravel())
        if np.max(np.abs(delta)) < 1e-5:
            break

        print('Relinearizing...')
        progress.update(r['x'][0], r['x'][1:7], r['x'][7:])

    if FLAGS.gui:
        errofx = casadi.Function('errofx', [x], [errvec])
        nex = np.sqrt(np.array(errofx.call([r['x']])[0]).ravel())
        progress.plot(errs=nex)

    progress.alignment.serialize()

    # IPython.embed()


if __name__ == '__main__':
    faulthandler.enable()
    sys.excepthook = ultratb.FormattedTB(
        mode='Plain', color_scheme='Linux', call_pdb=1)
    sys.argv = flags.FLAGS(sys.argv)
    if not FLAGS.gui:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import uzh_fpv.plots as plots
    run()
