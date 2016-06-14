#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Datatype for partitioning a secuence of poses executed by a instructor
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
from scipy.signal import argrelextrema

import configuration.constants as sysConstants
from parser.BvhImport import BvhImport

Y_THREADHOLD_SECURITY = -5


def find_nearest(a, a0):
    "Element in nd array `a` closest to the scalar value `a0`"
    idx = np.abs(a - a0).argmin()
    return a.flat[idx]


class DTMotorTaskProperty(object):
    def __init__(self):
        self.filename = ""
        self.start = 0
        self.end = 0
        self.Y_THREADHOLD = 11  # TODO calculate this as the average of the steps_highs
        self.X_THREADHOLD = 36
        self.framesPerSecond = 20  # TODO add this to configuration xml
        self.direction = sysConstants.RIGHT_TO_LEFT

    def getIndividualStart(self):
        return self.start

    def getIndividualEnd(self):
        return self.end

    def getMoveDirection(self):
        return self.direction


class DTWalkCycleProperty(DTMotorTaskProperty):
    def __init__(self, file):
        DTMotorTaskProperty.__init__(self)
        self.filename = file
        parser = BvhImport(self.filename)
        x_, y_, z_ = parser.getNodePositionsFromName("lFoot")
        y1 = []
        y2 = []
        x1 = []
        x2 = []

        for key, value in y_.iteritems():
            y1.append(value)
            x1.append(key)

        x_, y_, z_ = parser.getNodePositionsFromName("rFoot")
        for key, value in y_.iteritems():
            y2.append(value)
            x2.append(key)

        maxLfootIndexes = [x for x in argrelextrema(np.array(y1), np.greater)[0]]
        maxRfootIndexes = [x for x in argrelextrema(np.array(y2), np.greater)[0]]

        stepsLfootIndexes = []
        y_threadhold = self.Y_THREADHOLD
        while True:  # repeat until implementation in python
            for i in range(len(maxLfootIndexes)):
                index = maxLfootIndexes[i]
                if y1[index] - y2[index] > y_threadhold:  # one foot is up and the other is in the floor
                    if len(stepsLfootIndexes) > 0:
                        nearestStepIndex = find_nearest(np.array(stepsLfootIndexes), index)
                        if abs(index - nearestStepIndex) > self.X_THREADHOLD:  # avoid max near an existing point
                            stepsLfootIndexes.append(index)
                        else:
                            if y1[nearestStepIndex] < y1[index]:  # check if the exiting near max is a local maximun
                                stepsLfootIndexes.remove(nearestStepIndex)
                                stepsLfootIndexes.append(index)
                    else:
                        stepsLfootIndexes.append(index)

            if len(stepsLfootIndexes) > 0:
                break
            else:
                if y_threadhold > Y_THREADHOLD_SECURITY:
                    y_threadhold -= 1
                    print "new Y_THREADHOLD: ", y_threadhold

        stepsRfootIndexes = []
        y_threadhold = self.Y_THREADHOLD
        while True:  # repeat until implementation in python, until condition in the break statement
            for i in range(len(maxRfootIndexes)):
                index = maxRfootIndexes[i]
                if y2[index] - y1[index] > y_threadhold:  # one foot is up and the other is in the floor
                    if len(stepsRfootIndexes) > 0:
                        nearestStepIndex = find_nearest(np.array(stepsRfootIndexes), index)
                        if abs(index - nearestStepIndex) > self.X_THREADHOLD:  # avoid max near an existing point
                            stepsRfootIndexes.append(index)
                        else:
                            if y2[nearestStepIndex] < y2[index]:  # check if the exiting near max is a local maximun
                                stepsRfootIndexes.remove(nearestStepIndex)
                                stepsRfootIndexes.append(index)
                    else:
                        stepsRfootIndexes.append(index)
            if len(stepsRfootIndexes) > 0:
                break
            else:
                if y_threadhold > Y_THREADHOLD_SECURITY:
                    y_threadhold -= 1
                    print "new Y_THREADHOLD: ", y_threadhold

        if stepsLfootIndexes[0] < stepsRfootIndexes[0]:  # start walking with right leg
            testPoint = stepsLfootIndexes[0]
            while (y1[testPoint] > y2[testPoint]):
                testPoint = testPoint + 1

            self.end = testPoint + 15
        else:
            testPoint = stepsRfootIndexes[0]
            while (y2[testPoint] > y1[testPoint]):
                testPoint = testPoint + 1
            self.end = testPoint + 15

        absis_value = []
        absis_key = []

        for key, value in z_.iteritems():
            absis_value.append(value)
            absis_key.append(key)

        # direction of the walking cycle
        if absis_value[0] > absis_value[self.end]:
            self.direction = sysConstants.RIGHT_TO_LEFT
        else:
            self.direction = sysConstants.LEFT_TO_RIGHT


class DTWalkCycleStartingLeftFootProperty(DTMotorTaskProperty):
    # y1 and y2 are time serie vectors with the Y coordinate of each foot
    # the function returns the a set of indexes for which the y1 foot is over the y2 foot and the difference in the Y axis is maximum
    def findMaximunGapBetweenFoots(self, y1, y2):
        maxFootIndexes = [x for x in argrelextrema(np.array(y1), np.greater)[
            0]]  # each value of this vector corresponds to a local maximum of the y1 values
        maximumFootsGapIndexes = []
        y_threadhold = self.Y_THREADHOLD
        while True:  # repeat until implementation in python
            for i in range(len(maxFootIndexes)):
                index = maxFootIndexes[i]
                if y1[index] - y2[index] > y_threadhold:  # left foot is up and right foot on the floor
                    if len(maximumFootsGapIndexes) > 0:
                        nearestStepIndex = find_nearest(np.array(maximumFootsGapIndexes), index)
                        if abs(index - nearestStepIndex) > self.X_THREADHOLD:  # avoid max near an existing point
                            maximumFootsGapIndexes.append(index)
                        else:
                            if y1[nearestStepIndex] < y1[index]:  # check if the exiting near max is a local maximum
                                maximumFootsGapIndexes.remove(nearestStepIndex)
                                maximumFootsGapIndexes.append(index)
                    else:
                        maximumFootsGapIndexes.append(index)

            if len(maximumFootsGapIndexes) > 1:
                break
            else:
                if y_threadhold > Y_THREADHOLD_SECURITY:
                    y_threadhold -= 1
                    print "new Y_THREADHOLD: ", y_threadhold

        return maximumFootsGapIndexes

    def calculateWalkCycleStartingLeftFoot(self, parser):
        y1 = []
        y2 = []
        x1 = []
        x2 = []

        x_, y_, z_ = parser.getNodePositionsFromName(
            "lFoot")  # time series of (value, key) tuples for the lFoot node
        for key, value in y_.iteritems():
            y1.append(value)
            x1.append(key)

        x_, y_, z_ = parser.getNodePositionsFromName(
            "rFoot")  # time series of (value, key) tuples for the rFoot node
        for key, value in y_.iteritems():
            y2.append(value)
            x2.append(key)

        stepsLfootIndexes = self.findMaximunGapBetweenFoots(y1, y2)
        stepsRfootIndexes = self.findMaximunGapBetweenFoots(y2, y1)

        # start cycle condition: left foot backward and right foot forward

        leftFootPointerStepsIndex = 0
        rightFootPointerStepsIndex = 0
        if stepsLfootIndexes[leftFootPointerStepsIndex] < stepsRfootIndexes[rightFootPointerStepsIndex]:  # start walking with left leg
            testPoint = stepsLfootIndexes[leftFootPointerStepsIndex]
            leftFootPointerStepsIndex += 1
            while (y1[testPoint] > y2[testPoint]):  # wait while the left leg is on the air
                testPoint += 1
            testPoint = stepsRfootIndexes[rightFootPointerStepsIndex]
            rightFootPointerStepsIndex += 1
            while (y2[testPoint] > y1[testPoint]):  # wait while the right leg is on the air
                testPoint += 1
            self.start = testPoint  # end pre-cycle, start cycle

        else:  # start walking with right leg
            testPoint = stepsRfootIndexes[rightFootPointerStepsIndex]
            rightFootPointerStepsIndex += 1
            while (y2[testPoint] > y1[testPoint]):  # wait while the right leg is on the air
                testPoint += 1
            self.start = testPoint  # end pre-cycle start cycle


        # actual condition: left foot backward and right foot forward
        testPoint = stepsLfootIndexes[leftFootPointerStepsIndex]
        leftFootPointerStepsIndex += 1
        while (y1[testPoint] > y2[testPoint]):  # wait while the right leg is on the air
            testPoint += 1
        testPoint = stepsRfootIndexes[rightFootPointerStepsIndex]
        rightFootPointerStepsIndex+= 1
        while (y2[testPoint] > y1[testPoint]):  # wait while the left leg is on the air
            testPoint += 1
        self.end = testPoint + 15  # end of the cycle

        absis_value = []
        absis_key = []

        for key, value in z_.iteritems():
            absis_value.append(value)
            absis_key.append(key)

        # direction of the walking cycle
        if absis_value[0] > absis_value[self.end]:
            self.direction = sysConstants.RIGHT_TO_LEFT
        else:
            self.direction = sysConstants.LEFT_TO_RIGHT

    def __init__(self, file):
        DTMotorTaskProperty.__init__(self)
        self.filename = file
        parser = BvhImport(self.filename)
        self.calculateWalkCycleStartingLeftFoot(parser)

class DTWalkPreCycleProperty(DTMotorTaskProperty):
    # y1 and y2 are time serie vectors with the Y coordinate of each foot
    # the function returns the a set of indexes for which the y1 foot is over the y2 foot and the difference in the Y axis is maximum
    def findMaximunGapBetweenFoots(self, y1, y2):
        maxFootIndexes = [x for x in argrelextrema(np.array(y1), np.greater)[
            0]]  # each value of this vector corresponds to a local maximum of the y1 values
        maximumFootsGapIndexes = []
        y_threadhold = self.Y_THREADHOLD
        while True:  # repeat until implementation in python
            for i in range(len(maxFootIndexes)):
                index = maxFootIndexes[i]
                if y1[index] - y2[index] > y_threadhold:  # left foot is up and right foot on the floor
                    if len(maximumFootsGapIndexes) > 0:
                        nearestStepIndex = find_nearest(np.array(maximumFootsGapIndexes), index)
                        if abs(index - nearestStepIndex) > self.X_THREADHOLD:  # avoid max near an existing point
                            maximumFootsGapIndexes.append(index)
                        else:
                            if y1[nearestStepIndex] < y1[index]:  # check if the exiting near max is a local maximum
                                maximumFootsGapIndexes.remove(nearestStepIndex)
                                maximumFootsGapIndexes.append(index)
                    else:
                        maximumFootsGapIndexes.append(index)

            if len(maximumFootsGapIndexes) > 1:
                break
            else:
                if y_threadhold > Y_THREADHOLD_SECURITY:
                    y_threadhold -= 1
                    print "new Y_THREADHOLD: ", y_threadhold

        return maximumFootsGapIndexes

    def calculatePreCycle(self, parser):
        y1 = []
        y2 = []
        x1 = []
        x2 = []

        x_, y_, z_ = parser.getNodePositionsFromName(
            "lFoot")  # time series of (value, key) tuples for the lFoot node
        for key, value in y_.iteritems():
            y1.append(value)
            x1.append(key)

        x_, y_, z_ = parser.getNodePositionsFromName(
            "rFoot")  # time series of (value, key) tuples for the rFoot node
        for key, value in y_.iteritems():
            y2.append(value)
            x2.append(key)

        stepsLfootIndexes = self.findMaximunGapBetweenFoots(y1, y2)
        stepsRfootIndexes = self.findMaximunGapBetweenFoots(y2, y1)

        # start cycle condition: left foot backward and right foot forward

        leftFootPointerStepsIndex = 0
        rightFootPointerStepsIndex = 0
        self.start = 0

        if stepsLfootIndexes[leftFootPointerStepsIndex] < stepsRfootIndexes[rightFootPointerStepsIndex]:  # start walking with left leg
            testPoint = stepsLfootIndexes[leftFootPointerStepsIndex]
            leftFootPointerStepsIndex += 1
            while (y1[testPoint] > y2[testPoint]):  # wait while the left leg is on the air
                testPoint += 1
            testPoint = stepsRfootIndexes[rightFootPointerStepsIndex]
            rightFootPointerStepsIndex += 1
            while (y2[testPoint] > y1[testPoint]):  # wait while the right leg is on the air
                testPoint += 1
            self.end = testPoint  # end pre-cycle, start cycle

        else:  # start walking with right leg
            testPoint = stepsRfootIndexes[rightFootPointerStepsIndex]
            rightFootPointerStepsIndex += 1
            while (y2[testPoint] > y1[testPoint]):  # wait while the right leg is on the air
                testPoint += 1
            self.end = testPoint  # end pre-cycle start cycle

        absis_value = []
        absis_key = []

        for key, value in z_.iteritems():
            absis_value.append(value)
            absis_key.append(key)

        # direction of the walking cycle
        if absis_value[0] > absis_value[self.end]:
            self.direction = sysConstants.RIGHT_TO_LEFT
        else:
            self.direction = sysConstants.LEFT_TO_RIGHT

    def __init__(self, file):
        DTMotorTaskProperty.__init__(self)
        self.filename = file
        parser = BvhImport(self.filename)
        self.calculatePreCycle(parser)


