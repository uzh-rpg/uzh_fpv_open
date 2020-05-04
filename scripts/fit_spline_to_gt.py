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
import matplotlib
import numpy as np
import os
import sys

import uzh_fpv.bspline_opt as bspline_opt
import uzh_fpv.casadi_pose as casadi_pose
import uzh_fpv.flags as flags
import uzh_fpv.linear_interpolation as linear_interpolation
import uzh_fpv.pose as pose
import uzh_fpv.util as util


FLAGS = flags.FLAGS


def rotationAngle(mx_R):
    acosarg = (casadi.trace(mx_R) - 1.) / 2.
    return casadi.if_else(casadi.fabs(acosarg) >= 1., 0., casadi.acos(acosarg))


def errorAtTime(gt_traj, sym_traj, time, gt_pose=None):
    if gt_pose is None:
        gt_pose = gt_traj.sample(time)
    sym_pose = sym_traj.sample(time)
    if gt_pose is not None and sym_pose is not None:
        relpose = casadi_pose.mulNumCas(gt_pose.inverse(), sym_pose)
        return rotationAngle(relpose.R), casadi.norm_2(relpose.t), gt_pose, sym_pose
    else:
        return None, None, None, None


def numericTrajectoryError(valid_gt_poses, test_poses):
    terrs = np.array([np.linalg.norm(vgp.t - tp.t) for vgp, tp in zip(valid_gt_poses, test_poses)])
    mterr = terrs.mean()
    print('Mean position error is %f m' % mterr)

    Rerrs = np.array([np.rad2deg(pose.geodesicDistanceSO3(vgp.R, tp.R)) for vgp, tp in zip(valid_gt_poses, test_poses)])
    assert np.all(Rerrs >= 0.)
    mRerr = Rerrs.mean()
    print('Mean rotation error is %f degrees' % mRerr)

    return mRerr, mterr


def maxTwistNorm(x):
    return np.max(np.array([np.linalg.norm(i[3:]) for i in np.array(x).reshape((-1, 6))]))


def avgTwistNorm(x):
    return np.mean(np.array([np.linalg.norm(i[3:]) for i in np.array(x).reshape((-1, 6))]))


def run():
    print('Getting samples from bag...')
    gt_times, gt_poses, t_GT_spline = evaluate_discretizations.getSamplesFromBag()
    gt_traj = linear_interpolation.Trajectory(gt_times, gt_poses)

    print('Calculating initial guess...')
    traj, valid_times, valid_poses, valid_mask = \
        evaluate_discretizations.considerDiscretization(gt_times, gt_poses, FLAGS.dt)
    valid_gt_poses = [i for i, v in zip(gt_poses, valid_mask) if v]

    print('Formulating symbolic trajectory...')
    sym_traj = bspline_opt.SymbolicTrajectory(traj)

    print('Calculating error terms...')
    want_num_error_terms = FLAGS.errs
    step = (len(valid_times) / want_num_error_terms) + 1
    error_times = valid_times[::step]
    error_gt_poses = valid_gt_poses[::step]

    print('Using %d error times' % len(error_times))

    eRvec = None
    etvec = None
    gtposes = []
    symposes = []
    for t, gtps in zip(error_times, error_gt_poses):
        eR, et, gtp, syp = errorAtTime(gt_traj, sym_traj, t, gt_pose=gtps)
        if eR is not None:
            if eRvec is None:
                eRvec = eR
                etvec = et
            else:
                eRvec = casadi.vertcat(eRvec, eR)
                etvec = casadi.vertcat(etvec, et)
            gtposes.append(gtp)
            symposes.append(syp)

    print('Setting up problem...')
    nlp = {'x': sym_traj.xvec, 'f': casadi.sum1(casadi.vertcat(eRvec, etvec))}
    n_iter = 3
    ipopt_options = {'max_iter': n_iter}
    nlp_options = {'ipopt': ipopt_options, 'verbose': False}
    S = casadi.nlpsol('S', 'ipopt', nlp, nlp_options)

    print('3 iterations at a time...')

    iters = [0]
    print('Old trajectory:')
    mRerr, mterr = numericTrajectoryError(valid_gt_poses, valid_poses)
    mterrs = [mterr]
    mRerrs = [mRerr]

    x0 = np.zeros(sym_traj.xvec.shape)
    xs = [x0]
    for iter in range(10):
        r = S(x0=x0)
        new_traj = sym_traj.newTrajectory(r['x'])
        new_valid_poses = [new_traj.sample(i) for i in valid_times]
        iters.append((iter + 1) * n_iter)
        print('New trajectory, %d iters:' % iters[-1])
        mRerr, mterr = numericTrajectoryError(valid_gt_poses, new_valid_poses)

        mterrs.append(mterr)
        mRerrs.append(mRerr)
        x0 = r['x']
        xs.append(r['x'])

    files = flags.GtFittedSplineFiles()
    if not os.path.exists(files.root):
        os.makedirs(files.root)
    new_traj.serialize(files.spline)
    util.writeRostimeToFile(files.t_GT_spline, t_GT_spline)

    mtns = [maxTwistNorm(x) for x in xs]
    atns = [avgTwistNorm(x) for x in xs]

    # Axis 1: Relative error
    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel('iteration')
    ax1.set_ylabel('Relative error', color=color)
    ax1.semilogy(iters, [i / max(mterrs) for i in mterrs], color=color, label='translation')
    ax1.semilogy(iters, [i / max(mRerrs) for i in mRerrs], '--', color=color, label='rotation')
    ax1.legend()


    ax1.grid()

    ax2 = ax1.twinx()

    color = 'tab:blue'
    ax2.set_ylabel('twist norm (rotation)', color=color)
    ax2.plot(iters, mtns, color=color, label='max')
    ax2.plot(iters, atns, '--', color=color, label='mean')

    ax1.set_title('%s, dt=%.3f, %d nodes, %d t(err)s' %
                  (flags.sequenceSensorString(), FLAGS.dt, len(traj.node_times), len(error_times)))

    fig.tight_layout()  # otherwise the right y-label is slightly clipped

    if FLAGS.gui:
        plt.show()
    else:
        plt.savefig(os.path.join(flags.plotPath(), 'fit_spline_to_gt.pdf'),
                    bbox_inches='tight')


if __name__ == '__main__':
    faulthandler.enable()
    sys.excepthook = ultratb.FormattedTB(
        mode='Plain', color_scheme='Linux', call_pdb=1)
    sys.argv = flags.FLAGS(sys.argv)
    np.seterr(invalid='raise')
    if not FLAGS.gui:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import evaluate_discretizations
    run()
