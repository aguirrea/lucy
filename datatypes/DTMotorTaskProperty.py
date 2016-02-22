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

import matplotlib.pyplot as plt
import configuration.constants as sysConstants
import numpy as np

from parser.BvhImport import BvhImport
from configuration.LoadSystemConfiguration import LoadSystemConfiguration
from scipy.signal import argrelextrema
from collections import Counter


def find_nearest(a, a0):
    "Element in nd array `a` closest to the scalar value `a0`"
    idx = np.abs(a - a0).argmin()
    return a.flat[idx]

class DTMotorTaskProperty(object):
    def __init__(self):
        self.filename = ""
        self.end = 0
        self.Y_THREADHOLD = 11 #TODO calculate this as the average of the steps_highs
        self.X_THREADHOLD = 36
        self.framesPerSecond = 20 #TODO add this to configuration xml
        self.direction = sysConstants.RIGHT_TO_LEFT

    def getIndividualEnd(self):
        return self.end

    def getMoveDirection(self):
        return self.direction


class DTWalkCycleProperty(DTMotorTaskProperty):
    
    def __init__(self, file):
        DTMotorTaskProperty.__init__(self)
        self.filename = file
        parser = BvhImport(self.filename)
        x_,y_,z_ = parser.getNodePositionsFromName("lFoot")
        y1 = []
        y2 = []
        x1 = []
        x2 = []
        
        for key, value in y_.iteritems():
            y1.append(value)
            x1.append(key)
        
        x_,y_,z_ = parser.getNodePositionsFromName("rFoot")
        for key, value in y_.iteritems():
            y2.append(value)
            x2.append(key)
        
        maxLfootIndexes = [x for x in argrelextrema(np.array(y1), np.greater)[0]]
        maxRfootIndexes = [x for x in argrelextrema(np.array(y2), np.greater)[0]]

        stepsLfootIndexes = []
        for i in range(len(maxLfootIndexes)):
            index = maxLfootIndexes[i]
            if y1[index] - y2[index] > self.Y_THREADHOLD: #one foot is up and the other is in the floor  
                    if len(stepsLfootIndexes)>0:
                        if abs(index - find_nearest(np.array(stepsLfootIndexes), index) > self.X_THREADHOLD): #avoid max near an existing point
                            stepsLfootIndexes.append(index)
                        else:
                            if y1[find_nearest(np.array(stepsLfootIndexes), index)] < y1[index]:  #check if the exiting near max is a local maximun
                                stepsLfootIndexes.remove(find_nearest(np.array(stepsLfootIndexes), index))
                                stepsLfootIndexes.append(index)
                    else:
                        stepsLfootIndexes.append(index)
        
        stepsRfootIndexes = []
        for i in range(len(maxRfootIndexes)):
            index = maxRfootIndexes[i]
            if y2[index] - y1[index] > self.Y_THREADHOLD: #one foot is up and the other is in the floor
                    if len(stepsRfootIndexes)>0:
                        if abs(index - find_nearest(np.array(stepsRfootIndexes),index) > self.X_THREADHOLD): #avoid max near an existing point
                            stepsRfootIndexes.append(index)
                        else:
                            if y2[find_nearest(np.array(stepsRfootIndexes), index)] < y2[index]: #check if the exiting near max is a local maximun
                                stepsRfootIndexes.remove(find_nearest(np.array(stepsRfootIndexes), index))
                                stepsRfootIndexes.append(index)
            
                    else:
                        stepsRfootIndexes.append(index)

        absis_value = []
        absis_key = []

        for key, value in z_.iteritems():
            absis_value.append(value)
            absis_key.append(key)

        #direction of the walking cycle
        if absis_value[0] > absis_value[len(absis_value)-1]:
            print "aumenta"
            self.direction = sysConstants.LEFT_TO_RIGHT
        else:
            print "disminuye"
            self.direction = sysConstants.RIGHT_TO_LEFT

        if stepsLfootIndexes[0] < stepsRfootIndexes[0]: #start walking with right leg
            testPoint = stepsLfootIndexes[0]
            while(y1[testPoint]>y2[testPoint]):
                testPoint = testPoint + 1

            self.end = testPoint + 5
        else:
            testPoint = stepsRfootIndexes[0]
            while(y2[testPoint]>y1[testPoint]):
                testPoint = testPoint + 1
            self.end = testPoint + 5

