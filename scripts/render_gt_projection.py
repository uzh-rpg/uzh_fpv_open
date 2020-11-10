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

import absl
import csv
import cv2
import IPython
from IPython.core import ultratb
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
import yaml

import geometry_msgs.msg
import rospy
import tf2_ros

import uzh_fpv.calibration as calibration
import uzh_fpv.flags as flags
import uzh_fpv.pose as pose

absl.flags.DEFINE_string('estim', '', 'Use this estimate instead of the ground truth for rendering.')

FLAGS = flags.FLAGS


def importCamI(y, i):
    y_cam = y['cam%d' % i]
    T_C_I4 = np.array(y_cam['T_cam_imu'])
    T_C_B = pose.Pose(T_C_I4[:3, :3], T_C_I4[:3, 3:])
    dist = calibration.EquidistantDistortion(y_cam['distortion_coeffs'])
    intr = y_cam['intrinsics']
    shape = list(reversed(y_cam['resolution']))
    return calibration.CamCalibration(intr[:2], intr[2:], dist, shape, T_C_B)


if __name__ == '__main__':
    sys.excepthook = ultratb.FormattedTB(
        mode='Verbose', color_scheme='Linux', call_pdb=1)

    sys.argv = flags.FLAGS(sys.argv)
    rospy.init_node('bcast')
    br = tf2_ros.TransformBroadcaster()

    def bcastPose(T_W):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = 'bcast'
        t.transform.translation.x = T_W.t[0]
        t.transform.translation.y = T_W.t[1]
        t.transform.translation.z = T_W.t[2]
        q = T_W.q_wxyz()
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        br.sendTransform(t)

    if FLAGS.estim != '':
        render_path = os.path.join(flags.plotPath(), 'es_projection')
    else:
        render_path = os.path.join(flags.plotPath(), 'gt_projection')
    if not os.path.exists(render_path):
        os.makedirs(render_path)
    calib_path = flags.calibPath()

    # Load calibrations.
    f = open(os.path.join(calib_path, 'camchain-imucam-..%s_calib_%s_imu.yaml' % (flags.envCamString(), flags.sensorString())))
    y = yaml.load(f)
    calibs = [importCamI(y, i) for i in [0, 1]]

    # Load ground truth and image times:
    unz = flags.unzippedGtPath()
    if FLAGS.estim != '':
        gt = np.loadtxt(FLAGS.estim)
    else:
        gt = np.loadtxt(os.path.join(unz, 'groundtruth.txt'))[:, 1:]
    r = csv.reader(open(os.path.join(unz, 'left_images.txt')), delimiter=' ')
    l_img = list(r)
    gtt = gt[:, 0]
    imt = np.array([float(i[1]) for i in l_img[1:]])

    # Create grid of ground points:
    xymax = np.max(gt[:, 1:3], axis=0)
    xymin = np.min(gt[:, 1:3], axis=0)
    xr = np.arange(xymin[0], xymax[0], 0.5)
    yr = np.arange(xymin[1], xymax[1], 0.5)
    pts = np.array([[x, y, gt[0, 3] - 0.1] for x in xr for y in yr])

    out_i = 0
    im_i = 0
    gt_i = 0
    while im_i < len(imt) and gt_i < len(gtt):
        # Advance image until closest:
        while im_i + 1 < len(imt) and abs(imt[im_i + 1] - gtt[gt_i]) <= abs(imt[im_i] - gtt[gt_i]):
            im_i = im_i + 1
        # Advance gt until closest:
        # leq, because some participants give subsequent estimates with same time.
        while gt_i + 1 < len(gtt) and abs(imt[im_i] - gtt[gt_i + 1]) <= abs(imt[im_i] - gtt[gt_i]):
            gt_i = gt_i + 1
        if im_i == len(imt) or gt_i == len(gtt) - 1:
            break

        cv2.imread(os.path.join(unz, l_img[im_i][2]))
        im = cv2.imread(os.path.join(unz, l_img[im_i][2]))

        q_xyzw = gt[gt_i, 4:]
        q_wxyz = np.hstack((q_xyzw[3], q_xyzw[:3]))
        T_W_B = pose.fromPositionAndQuaternion(gt[gt_i, 1:4], q_wxyz)
        T_C_W = calibs[0].T_C_B * T_W_B.inverse()
        bcastPose(T_W_B)
        p_C = T_C_W * pts.T

        prj_rc = calibs[0].project(p_C.T)
        plt.clf()
        plt.imshow(im)
        plt.plot(prj_rc[:, 1], prj_rc[:, 0], 'r.')
        plt.xlim((0, im.shape[1]))
        plt.ylim((im.shape[0], 0))
        plt.savefig(os.path.join(render_path, '%06d.png' % out_i))
        print("%d, im time %f, est time %f" % (out_i, imt[im_i], gtt[gt_i]))
        out_i = out_i + 1
        im_i = im_i + 1
