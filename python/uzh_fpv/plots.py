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


def xyPlot(imu_positions, prism_positions, leica_measurements):
    if imu_positions is not None:
        plt.plot(imu_positions[:, 0], imu_positions[:, 1], label='IMU trajectory')
    if prism_positions is not None:
        plt.plot(prism_positions[:, 0], prism_positions[:, 1], label='prism trajectory')
    if leica_measurements is not None:
        plt.plot(leica_measurements[:, 0], leica_measurements[:, 1], '.', label='leica measurements')
    plt.grid()
    plt.legend()
    plt.gca().set_aspect('equal')
    plt.xlabel('x')
    plt.ylabel('y')


def xyztPlot(times, imu_positions, prism_positions, leica_times, leica_measurements):
    plt.plot(3 * leica_times, leica_measurements.T.ravel(), '.', label='leica')
    plt.plot(times, imu_positions[:, 0], label='IMU x')
    plt.plot(times, imu_positions[:, 1], label='IMU y')
    plt.plot(times, imu_positions[:, 2], label='IMU z')

    plt.plot(times, prism_positions[:, 0], label='prism x')
    plt.plot(times, prism_positions[:, 1], label='prism y')
    plt.plot(times, prism_positions[:, 2], label='prism z')
    plt.grid()
    plt.legend()
    plt.xlabel('t')
    plt.ylabel('x, y, z')
    # plt.xlim([times[0] - 1, times[-1] + 1])
