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

import datetime
import IPython
import numpy as np
import re
import rospy
import time

from generalized_time import GeneralizedTime


def parseTextFile(path):
    leica_file = open(path, 'r')

    # Patterns:
    date_time = '\d\d\d\d-\d\d-\d\d \d\d:\d\d:\d\d.\d\d\d'
    value = '-?\d+\.\d+'

    op_line = '^0,%s,Operator,.*$' % date_time
    empty_line = '^[\r\n]?$'
    start_line = '^0,%s,Start![\r\n]?$' % date_time
    weird_line = '^1,%s,%%R1Q,0,\d:,%s,%%R1P,0,\d:0[\r\n]?$' % (date_time, date_time)
    meas_line = re.compile('^3,(%s),%%R1Q,2082,\d:10000,1,%s,%%R1P,0,\d:(\d+),(%s),(%s),(%s),\d+,%s,%s,%s,\d+[\r\n]?$'
                           % (date_time, date_time, value, value, value, value, value, value))
    outlier_line = '^3,%s,%%R1Q,2082,\d:10000,1,%s,%%R1P,0,\d:\d+,0,0,0,0,0,0,0,0[\r\n]?$' % (
        date_time, date_time)
    final_line = '^0,%s,Canceled![\r\n]?$' % date_time

    t = []
    x = []
    y = []
    z = []
    for line in leica_file:
        meas_match = meas_line.match(line)
        if meas_match:
            if meas_match.group(2) in ['0', '1284']:
                dt = datetime.datetime.strptime(meas_match.group(1) + '000', '%Y-%m-%d %H:%M:%S.%f')
                # Could also use ROS Time here, or any other time representation.
                t.append(GeneralizedTime(int(time.mktime(dt.timetuple())), dt.microsecond * 1000))
                x.append(float(meas_match.group(3)))
                y.append(float(meas_match.group(4)))
                z.append(float(meas_match.group(5)))
        elif re.match(op_line, line) or re.match(empty_line, line) or re.match(start_line, line) or \
                re.match(weird_line, line) or re.match(final_line, line) or re.match(outlier_line, line):
            pass
        else:
            print('The following line does not match any pattern:')
            print(line)
            IPython.embed()

    return t, np.array([x, y, z]).T
