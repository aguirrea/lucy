#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Fitness function Factory
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

from datatypes.DTIndividualProperty import DTIndividualPropertyVanillaEvolutive

from LoadRobotConfiguration         import LoadRobotConfiguration

ORACLE_DISTANCE = 0.19 #Baliero & Pías work, dist_ini + 2 * dist_paso

class FitnessFunction(object):
    def __init__(self, dtFitness):
        self.parameters = dtFitness
        self.fitness = 0

    def getFitness(self):
        if self.fitness < 0:
            return 0
        else:
            return self.fitness

    def normaliseConcatenationGap(self, concatenationGap):
        prop = DTIndividualPropertyVanillaEvolutive()
        robotConf = LoadRobotConfiguration()
        robotJoints = robotConf.getJointsName()
        totalJointsQty = len(robotJoints)
        avoidJointsQty = 0
        maxJointDiff = 300
        for joint in robotJoints:
            if prop.diffAvoidJoint(joint):
                avoidJointsQty += 1
        minDiff = ((totalJointsQty - avoidJointsQty) ** 2) * maxJointDiff
        maxDiff = 0
        normalizedGap = (concatenationGap - minDiff) / (maxDiff - minDiff)
        return abs(normalizedGap)

    def normaliseDistance(self, rawDistance):
        normDistance = rawDistance / ORACLE_DISTANCE
        if normDistance < 1:
            return normDistance
        else:
            return 1

    def normaliseAngle(self, rawAngle):
        minAngle = 1.57
        maxAngle = 0
        normalizedAngle = abs((rawAngle-minAngle)/(maxAngle-minAngle))
        if normalizedAngle > 1:
            print "*****************************************normalizedAngle:", normalizedAngle
            normalizedAngle = 1
        return normalizedAngle

class DistanceConcatenationgapFramesexecutedEndcyclebalanceAngle(FitnessFunction):
    def __init__(self, dtFitness):
        FitnessFunction.__init__(self, dtFitness)
        self.distanceWeight = 0.20
        self.concatenationGapNormalizedWeight = 0.20
        self.framesExecutedWeight = 0.30
        self.endCycleBalanceWeight = 0.30
        self.fitness = self.distanceWeight * self.parameters.getDistance()**(1/4.0) + self.concatenationGapNormalizedWeight * self.normaliseConcatenationGap(self.parameters.getConcatenationGap()) ** 6 + self.framesExecutedWeight * self.parameters.getFramesExecuted() + self.endCycleBalanceWeight * self.parameters.getEndCycleBalance() ** 6 - abs(self.parameters.getAngle())
        print "concatenationGapNormalized: ", self.normaliseConcatenationGap(self.parameters.getConcatenationGap())

class ConcatenationgapFramesexecutedNormAngle(FitnessFunction):
    def __init__(self, dtFitness):
        FitnessFunction.__init__(self, dtFitness)
        self.concatenationGapNormalizedWeight = 0.25
        self.framesExecutedWeight = 0.5
        self.normAngleWeight = 0.25
        self.fitness = self.concatenationGapNormalizedWeight * self.normaliseConcatenationGap(self.parameters.getConcatenationGap()) ** 6 + self.framesExecutedWeight * self.parameters.getFramesExecuted() ** 6 + self.normAngleWeight * self.normaliseAngle(self.parameters.getAngle())
        print "concatenationGapNormalized: ", self.normaliseConcatenationGap(self.parameters.getConcatenationGap())

class NormdistanceConcatenationgapFramesexecutedNormAngle(FitnessFunction):
    def __init__(self, dtFitness):
        FitnessFunction.__init__(self, dtFitness)
        self.distanceWeight = 0
        self.concatenationGapNormalizedWeight = 0.25
        self.framesExecutedWeight = 0.5
        self.normAngleWeight = 0.25
        self.fitness = self.distanceWeight * self.normaliseDistance(self.parameters.getDistance()) + self.concatenationGapNormalizedWeight * self.normaliseConcatenationGap(self.parameters.getConcatenationGap()) ** 6 + self.framesExecutedWeight * self.parameters.getFramesExecuted() ** 6 + self.normAngleWeight * self.normaliseAngle(self.parameters.getAngle())
        print "concatenationGapNormalized: ", self.normaliseConcatenationGap(self.parameters.getConcatenationGap())
