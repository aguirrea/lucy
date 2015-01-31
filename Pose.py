#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Pose representation for calculus
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

import math

class Pose:

    def __init__(self, poseValues):
        self.value = poseValues

    def getValue(self, key):
        return self.value[key]

    def diff(self, pose):
        diff = 0
        for key in range(self.keys()):
            diff = diff + math.fabs(pose.getValue(key) - self.value[key])
        return diff


