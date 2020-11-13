# Copyright (C) 2020 Giovanni Cioffi, RPG, University of Zurich, Switzerland
#   You can contact the author at <cioffi at ifi dot uzh dot ch>
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


import argparse
import os

from matplotlib import pyplot as plt
import numpy as np

def plot(snappy_gt, davis_gt, dt):
    fig1, axs1 = plt.subplots(3)
    fig1.suptitle('Before matching')
    
    axs1[0].plot(snappy_gt[:, 0] - snappy_gt[0, 0], snappy_gt[:, 1], 'b-', label='snappy')
    axs1[0].plot(davis_gt[:, 0] - davis_gt[0, 0], davis_gt[:, 1], 'r-', label='davis')
    axs1[0].set_ylabel('x')
    axs1[0].set_xlabel('t')
    axs1[0].legend()

    axs1[1].plot(snappy_gt[:, 0] - snappy_gt[0, 0], snappy_gt[:, 2], 'b-', label='snappy')
    axs1[1].plot(davis_gt[:, 0] - davis_gt[0, 0], davis_gt[:, 2], 'r-', label='davis')
    axs1[1].set_ylabel('y')
    axs1[1].set_xlabel('t')
    axs1[1].legend()

    axs1[2].plot(snappy_gt[:, 0] - snappy_gt[0, 0], snappy_gt[:, 3], 'b-', label='snappy')
    axs1[2].plot(davis_gt[:, 0] - davis_gt[0, 0] , davis_gt[:, 3], 'r-', label='davis')
    axs1[2].set_ylabel('z')
    axs1[2].set_xlabel('t')
    axs1[2].legend()

    # Match traj. and compute num. of samples to crop.
    ts_davis_gt_matched = davis_gt[:, 0] - davis_gt[0, 0] - dt

    n_samples_to_crop_front = np.shape(ts_davis_gt_matched)[0]
    idxs = ts_davis_gt_matched > 0
    ts_davis_gt_matched = ts_davis_gt_matched[idxs]
    n_samples_to_crop_front -= np.shape(ts_davis_gt_matched)[0]
    
    n_samples_to_crop_back = np.shape(ts_davis_gt_matched)[0]
    idxs = ts_davis_gt_matched < (snappy_gt[-1,0] - snappy_gt[0,0])
    ts_davis_gt_matched = ts_davis_gt_matched[idxs]
    n_samples_to_crop_back -= np.shape(ts_davis_gt_matched)[0]
    
    if n_samples_to_crop_back > 0:
        x_davis_gt_matched = davis_gt[n_samples_to_crop_front: -n_samples_to_crop_back, 1]
        y_davis_gt_matched = davis_gt[n_samples_to_crop_front: -n_samples_to_crop_back, 2]
        z_davis_gt_matched = davis_gt[n_samples_to_crop_front: -n_samples_to_crop_back, 3]
    else:
        x_davis_gt_matched = davis_gt[n_samples_to_crop_front:, 1]
        y_davis_gt_matched = davis_gt[n_samples_to_crop_front:, 2]
        z_davis_gt_matched = davis_gt[n_samples_to_crop_front:, 3]

    fig2, axs2 = plt.subplots(3)
    fig2.suptitle('After matching')
    
    axs2[0].plot(snappy_gt[:, 0] - snappy_gt[0, 0], snappy_gt[:, 1], 'b-', label='snappy')
    axs2[0].plot(ts_davis_gt_matched, x_davis_gt_matched, 'r-', label='davis')
    axs2[0].set_ylabel('x')
    axs2[0].set_xlabel('t')
    axs2[0].legend()

    axs2[1].plot(snappy_gt[:, 0] - snappy_gt[0, 0], snappy_gt[:, 2], 'b-', label='snappy')
    axs2[1].plot(ts_davis_gt_matched, y_davis_gt_matched, 'r-', label='davis')
    axs2[1].set_ylabel('y')
    axs2[1].set_xlabel('t')
    axs2[1].legend()

    axs2[2].plot(snappy_gt[:, 0] - snappy_gt[0, 0], snappy_gt[:, 3], 'b-', label='snappy')
    axs2[2].plot(ts_davis_gt_matched, z_davis_gt_matched, 'r-', label='davis')
    axs2[2].set_ylabel('z')
    axs2[2].set_xlabel('t')
    axs2[2].legend()

    plt.show()

    print("t_start DAVIS: %.9f" % (ts_davis_gt_matched[0] + davis_gt[0,0] + dt))
    print("t_end DAVIS: %.9f" % (ts_davis_gt_matched[-1] + davis_gt[0,0] + dt))
    print("n_samples_to_crop_front: %d" % n_samples_to_crop_front)
    print("n_samples_to_crop_back: %d" % n_samples_to_crop_back)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='''Plot snapdragon vs davis gt trajectories''')
    parser.add_argument(
        '--snapdragon_gt', help='Snapdragon groundtruth')
    parser.add_argument(
        '--davis_gt', help="DAVIS groundtruth")
    parser.add_argument(
        '--dt', help="Time offset", type=float, default=0.0)
    
    args = parser.parse_args()
    assert os.path.exists(args.snapdragon_gt), "Snapdragon trajectory not found."
    assert os.path.exists(args.davis_gt), "DAVIS trajectory not found."

    # Groundtruth trajectories are in the same format as required here: https://github.com/uzh-rpg/rpg_trajectory_evaluation
    snappy_gt = np.loadtxt(args.snapdragon_gt)
    davis_gt = np.loadtxt(args.davis_gt)
    dt = args.dt

    plot(snappy_gt, davis_gt, dt)

