#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Datatype for the individual genetic material
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


import numpy as np
from scipy.interpolate import UnivariateSpline

import configuration.constants as sysConstants
from parser.LoadPoses import LoadPoses
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

SPLINE_SMOOTHING_FACTOR = 5
INTERPOLATION_WINDOW = 5
REFERENCE_WINDOW_RADIUS = 10

class DTIndividualGeneticMaterial(object):
    def __init__(self):
        self.geneticMatrix = []

    def getGeneticMatrix(self):
        return self.geneticMatrix

    def equalSentinelFrame(self, frame):
        sentinelFramePresent = True
        for jointValue in frame:
            sentinelFramePresent = sentinelFramePresent and jointValue == sysConstants.JOINT_SENTINEL
        return sentinelFramePresent

    def getLength(self):
        iter = 0
        genomaRawLength = len(self.geneticMatrix)
        while iter < genomaRawLength and not self.equalSentinelFrame(self.geneticMatrix[iter]):
            iter = iter + 1
        length = iter
        return length

    def concatenate(self, individualGeneticMaterial):
        self.geneticMatrix += individualGeneticMaterial.getGeneticMatrix()

    def repeat(self, times):
        self.geneticMatrix *= times


class DTIndividualGeneticTimeSerieFile(DTIndividualGeneticMaterial):
    def __init__(self, geneticMaterial):
        DTIndividualGeneticMaterial.__init__(self)
        lp = LoadPoses(geneticMaterial)
        robotConfig = LoadRobotConfiguration()
        poseSize = lp.getFrameQty()
        # newGenoma = G2DList.G2DList(poseSize, 18)
        # for i in xrange(newGenoma.getHeight()):
        #    for j in xrange(newGenoma.getWidth()):
        #        newGenoma.setItem(i, j, lp.getPose(i).getValue(j))
        # self.geneticMatrix = newGenoma
        self.geneticMatrix = [[lp.getPose(i).getValue(j) for j in robotConfig.getJointsName()] for i in
                              xrange(poseSize)]


class DTIndividualGeneticMatrix(DTIndividualGeneticMaterial):
    # def __init__(self, geneticMaterial=G2DList.G2DList(1,18)):
    def __init__(self, geneticMaterial=[[0 for j in xrange(18)] for i in xrange(1)]):
        DTIndividualGeneticMaterial.__init__(self)
        self.geneticMatrix = geneticMaterial


class DTIndividualGeneticTimeSerieFileWalk(DTIndividualGeneticMaterial):
    def __init__(self, geneticMaterial):
        CYCLE_REPETITION = 2
        DTIndividualGeneticMaterial.__init__(self)
        lp = LoadPoses(geneticMaterial)
        robotConfig = LoadRobotConfiguration()
        cycleSize = lp.getFrameQty()
        '''self.geneticMatrix = [[lp.getPose(i % cycleSize).getValue(j) for j in robotConfig.getJointsName()] for i
                              in xrange(poseSize)]'''
        self.geneticMatrix = [[lp.getPose(i).getValue(j) for j in robotConfig.getJointsName()] for i in
                              xrange(cycleSize)] * CYCLE_REPETITION
        print self.geneticMatrix

        x = np.ndarray(REFERENCE_WINDOW_RADIUS * 2)
        y = np.ndarray(REFERENCE_WINDOW_RADIUS * 2)

        poseQty = len(self.geneticMatrix)
        poseLength = len(self.geneticMatrix[0])
        print "poseQty: ", poseQty, "poseLength: ", poseLength, "lp.getFrameQty(): ", lp.getFrameQty()


        for joint in range(poseLength):
            interpolationDataIter = REFERENCE_WINDOW_RADIUS

            for k in xrange(REFERENCE_WINDOW_RADIUS):
                referenceFrame= cycleSize + k
                x[interpolationDataIter] = referenceFrame
                y[interpolationDataIter] = self.geneticMatrix[referenceFrame][joint]
                interpolationDataIter += 1

            interpolationDataIter = REFERENCE_WINDOW_RADIUS - 1

            for k in xrange(REFERENCE_WINDOW_RADIUS):
                referenceFrame= cycleSize - (INTERPOLATION_WINDOW + k + 1)
                x[interpolationDataIter] = referenceFrame
                y[interpolationDataIter] = self.geneticMatrix[referenceFrame][joint]
                interpolationDataIter -= 1

            spl = UnivariateSpline(x, y, s=SPLINE_SMOOTHING_FACTOR)

            print "interopArrayX: ", x, "interopArrayY: ", y
            for k in xrange(INTERPOLATION_WINDOW):
                smoothFrameIter = cycleSize - 1 - k
                newValue = spl(smoothFrameIter)
                print "joint: ", joint, "iter: ", smoothFrameIter, "old value: ", self.geneticMatrix[smoothFrameIter][joint], "new value: ", newValue
                self.geneticMatrix[smoothFrameIter][joint] = newValue



class DTIndividualGeneticMatrixWalk(DTIndividualGeneticMaterial):
    def __init__(self, geneticMaterial):
        DTIndividualGeneticMaterial.__init__(self)
        robotConfig = LoadRobotConfiguration()
        walkCycleCounter = 0
        noSentinelFound = True
        geneticMaterialLength = 0
        cycleRepetitionQuantity = 2
        while noSentinelFound:
            if super(DTIndividualGeneticMatrixWalk, self).equalSentinelFrame(geneticMaterial[geneticMaterialLength]):
                noSentinelFound = False
            else:
                geneticMaterialLength = geneticMaterialLength + 1

        newLength = geneticMaterialLength * cycleRepetitionQuantity + 1  # +1 for the sentinel value
        self.geneticMatrix = [[-1 for j in xrange(robotConfig.getJointQuantity())] for i in xrange(newLength)]
        frameIter = 0
        while walkCycleCounter < cycleRepetitionQuantity:
            for jointIter in range(robotConfig.getJointQuantity()):
                data = geneticMaterial[frameIter][jointIter]
                if data != sysConstants.JOINT_SENTINEL:
                    self.geneticMatrix[frameIter + (geneticMaterialLength - 1) * walkCycleCounter][jointIter] = data
                else:
                    walkCycleCounter = walkCycleCounter + 1
                    frameIter = 0
                    break
            frameIter = frameIter + 1
