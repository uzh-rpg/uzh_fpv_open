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

import math


class GeneralizedTime(object):
    """ ROS time's typing and positivity check is too restrictive. """
    def __init__(self, secs_or_time_obj, nsecs=None):
        """ Secs and nsecs may be negative. """
        if hasattr(secs_or_time_obj, 'secs'):
            assert hasattr(secs_or_time_obj, 'nsecs')
            self.__init__(secs_or_time_obj.secs, secs_or_time_obj.nsecs)
        elif type(secs_or_time_obj) == float:
            dec, i = math.modf(secs_or_time_obj)
            self.__init__(int(i), int(dec * 1e9))
        else:
            assert nsecs is not None
            assert abs(nsecs) < 1e9
            self.secs = secs_or_time_obj
            self.nsecs = nsecs

    def to_sec(self):
        return self.secs + float(self.nsecs) / 1e9

    def __add__(self, other):
        secs = self.secs + other.secs
        nsecs = self.nsecs + other.nsecs
        if abs(nsecs) >= 1e9:
            if nsecs > 0:
                nsecs = nsecs - 1e9
                secs = secs + 1
            else:
                nsecs = nsecs + 1e9
                secs = secs - 1
        return GeneralizedTime(secs, nsecs)

    def __sub__(self, other):
        return self.__add__(-GeneralizedTime(other))

    def __neg__(self):
        return GeneralizedTime(-self.secs, -self.nsecs)
