#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Datatype for fitness function parameters
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

class DTFitness(object):

    def __init__(self, distance=0, concatenationGap=0, framesExecuted=0, endCycleBalance=0, angle=0, cycleEnded=0):
        self.distance = distance
        self.concatenationGap = concatenationGap
        self.framesExecuted = framesExecuted
        self.endCycleBalance = endCycleBalance
        self.angle = angle
        self.cycleEnded = cycleEnded

    def getDistance(self):
        return self.distance

    def getConcatenationGap(self):
        return self.concatenationGap

    def getFramesExecuted(self):
        return self.framesExecuted

    def getEndCycleBalance(self):
        return self.endCycleBalance

    def getAngle(self):
        return self.angle

    def getCycleEnded(self):
        return self.cycleEnded