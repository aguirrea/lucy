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
from numpy import linspace
from scipy.interpolate import UnivariateSpline

import matplotlib.pyplot as plt

import configuration.constants as sysConstants
from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
from datatypes.DTGenomeFunctions import DTGenomeFunctions
from parser.LoadPoses import LoadPoses
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

SPLINE_SMOOTHING_FACTOR = 5
INTERPOLATION_WINDOW = 6
REFERENCE_WINDOW_RADIUS = 10

class DTIndividualGeneticMaterial(object):
    def __init__(self):
        conf = LoadSystemConfiguration()
        self.geneticMatrix = []
        self.cyclesQty = int(conf.getProperty("Concatenate walk cycles?"))
        self.robotConfig = LoadRobotConfiguration()
        self.jointNameIDMapping = {}
        jointIDCounter = 0
        for j in self.robotConfig.getJointsName():
            self.jointNameIDMapping[jointIDCounter] = j
            jointIDCounter+=1

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

    def getConcatenationGap(self):
        dtgf = DTGenomeFunctions()
        return dtgf.rawDiff(self.geneticMatrix[0], self.geneticMatrix[self.getLength() - 1])

    # |...........(referenceWindowsRadius)(interpolationWindow)|(referenceWindowsRadius)........|
    #calculates the set of the last points of the cycle (of size interpolationWindow) using as refence for the spline
    #the set of referenceWindowsRadius size points before the interpolationWindow and the referenceWindowsRadius size
    #set of the first points of the cycle. It does this for all the cyckeRepetition gaps in the concatenation of the
    #cycle
    def calculateGapByInterpolation(self, referenceWindowRadius, interpolationWindow, splineSmoothingFactor, cycleSize, cycleRepetition, graphicalRepresentation=False):

        x = np.ndarray(referenceWindowRadius * 2)
        y = np.ndarray(referenceWindowRadius * 2)

        poseQty = len(self.geneticMatrix)
        poseLength = len(self.geneticMatrix[0])
        #print "poseQty: ", poseQty, "poseLength: ", poseLength, "lp.getFrameQty(): ", lp.getFrameQty()


        for joint in range(poseLength):
            interpolationDataIter = referenceWindowRadius - 1

            for k in xrange(referenceWindowRadius):
                referenceFrame= cycleSize + k
                x[interpolationDataIter] = referenceFrame
                y[interpolationDataIter] = self.geneticMatrix[referenceFrame][joint]
                interpolationDataIter += 1

            interpolationDataIter = referenceWindowRadius - 2

            for k in xrange(referenceWindowRadius):
                referenceFrame= cycleSize - (interpolationWindow + k + 1)
                x[interpolationDataIter] = referenceFrame
                y[interpolationDataIter] = self.geneticMatrix[referenceFrame][joint]
                interpolationDataIter -= 1

            spl = UnivariateSpline(x, y, s=splineSmoothingFactor)

            if graphicalRepresentation:
                px = linspace(x[0], x[len(x)-1], len(x))
                py = spl(px)
                plt.plot(x, y, '.-')
                plt.plot(px, py)

                xinter = np.ndarray(interpolationWindow)
                yinter = np.ndarray(interpolationWindow)

                for k in xrange(interpolationWindow):
                    smoothFrameIter = cycleSize - 1 - k
                    xinter[k] = smoothFrameIter
                    yinter[k] = self.geneticMatrix[smoothFrameIter][joint]

                plt.plot(xinter, yinter)
                plt.title(self.jointNameIDMapping[joint])
                plt.show()
                print "gap between first and last: ", self.getConcatenationGap()

            for i in xrange(cycleRepetition - 1):
                for k in range(cycleSize - (interpolationWindow + referenceWindowRadius), cycleSize + referenceWindowRadius):
                    newValue = spl(k)
                    self.geneticMatrix[cycleSize*i + k][joint] = newValue
                    if i == cycleRepetition - 2: #am I in the last step?
                        if k < cycleSize:        #if i am in the last step, is not necessary to remplace data beyond this cycle
                            self.geneticMatrix[cycleSize * i + k][joint] = newValue


class DTIndividualGeneticTimeSerieFile(DTIndividualGeneticMaterial):
    def __init__(self, geneticMaterial):
        DTIndividualGeneticMaterial.__init__(self)
        lp = LoadPoses(geneticMaterial)
        robotConfig = LoadRobotConfiguration()
        poseSize = lp.getFrameQty()
        self.geneticMatrix = [[lp.getPose(i).getValue(j) for j in robotConfig.getJointsName()] for i in
                              xrange(poseSize)]

class DTIndividualGeneticMatrix(DTIndividualGeneticMaterial):
    # def __init__(self, geneticMaterial=G2DList.G2DList(1,18)):
    def __init__(self, geneticMaterial=[[0 for j in xrange(18)] for i in xrange(1)]):
        DTIndividualGeneticMaterial.__init__(self)
        self.geneticMatrix = geneticMaterial

class DTIndividualGeneticTimeSerieFileWalk(DTIndividualGeneticMaterial):
    def __init__(self, geneticMaterial):
        DTIndividualGeneticMaterial.__init__(self)
        
        lp = LoadPoses(geneticMaterial)
        cycleSize = lp.getFrameQty()
        CYCLE_REPETITION = self.cyclesQty
        print "cicleSize: ", cycleSize
        self.geneticMatrix = [[lp.getPose(i).getValue(j) for j in self.robotConfig.getJointsName()] for i in
                              xrange(cycleSize)] * CYCLE_REPETITION
        #for debugging info:
        #self.calculateGapByInterpolation(REFERENCE_WINDOW_RADIUS, INTERPOLATION_WINDOW, SPLINE_SMOOTHING_FACTOR, cycleSize, CYCLE_REPETITION, True)
        self.calculateGapByInterpolation(REFERENCE_WINDOW_RADIUS, INTERPOLATION_WINDOW, SPLINE_SMOOTHING_FACTOR, cycleSize, CYCLE_REPETITION)


class DTIndividualGeneticMatrixWalk(DTIndividualGeneticMaterial):
    def __init__(self, geneticMaterial):
        DTIndividualGeneticMaterial.__init__(self)
        
        self.geneticMatrix = geneticMaterial
        CYCLE_REPETITION = self.cyclesQty
        cycleSize = self.getLength()
        print "cicleSize: ", cycleSize
        self.geneticMatrix = [[self.geneticMatrix[i][j] for j in xrange(len(self.jointNameIDMapping))] for i in
                              xrange(cycleSize)] * CYCLE_REPETITION
        #for debugging info:
        #self.calculateGapByInterpolation(REFERENCE_WINDOW_RADIUS, INTERPOLATION_WINDOW, SPLINE_SMOOTHING_FACTOR, cycleSize, CYCLE_REPETITION, True)
        self.calculateGapByInterpolation(REFERENCE_WINDOW_RADIUS, INTERPOLATION_WINDOW, SPLINE_SMOOTHING_FACTOR, cycleSize, CYCLE_REPETITION)

