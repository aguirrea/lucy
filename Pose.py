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

from datatypes.DTModelRepose import DTModelVrepReda
from simulator.LoadRobotConfiguration import LoadRobotConfiguration


class Pose:

    def __init__(self, poseValues={}):
        self.value = poseValues
        self.modelReposeValue=DTModelVrepReda()
        configuration = LoadRobotConfiguration()
        for joint in configuration.getJointsName():
            if joint not in self.value.keys():
                self.value[joint]=self.modelReposeValue.getReposeValue(joint)
                #print "default value for joint: ", joint, " is: ", self.value[joint]

    def setValue(self, key, value):
        self.value[key] = value

    def getValue(self, key):
        if key in self.value.keys():
            return self.value[key]
        else: #it can't happen
            return self.modelReposeValue.getReposeValue(key)

    def diff(self, pose):
        diff = 0
        for key in xrange(self.keys()):
            diff = diff + math.fabs(pose.getValue(key) - self.value[key])
        return diff


