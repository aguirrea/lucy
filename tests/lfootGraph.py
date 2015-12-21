#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
#
# MINA/INCO/UDELAR
#
# module for finding the steps in the tutors
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

import os
import glob
import ntpath

from parser.BvhImport import BvhImport
import matplotlib.pyplot as plt
from configuration.LoadSystemConfiguration import LoadSystemConfiguration
import numpy as np
from scipy.signal import argrelextrema
from collections import Counter

sysConf = LoadSystemConfiguration()
BVHDir = os.getcwd() + sysConf.getDirectory("CMU mocap Files")

Y_THREADHOLD = 11 #TODO calculate this as the average of the steps_highs
X_THREADHOLD = 36


def firstMax(values1, values2):
    res=0
    for i in range(len(values1)-2):
        if values1[i] < values1[i+1] and values1[i+1] > values1[i+2]: #i+1 is a local maximun
            if (values1[i] - values2[i]) > THREADHOLD:  
                res=i+1
        elif values1[i] < values1[i+1] < values1[i+2]: #i is a local maximun
            if (values1[i] - values2[i]) > THREADHOLD:  
                res=i
    return res

def find_nearest(a, a0):
    "Element in nd array `a` closest to the scalar value `a0`"
    idx = np.abs(a - a0).argmin()
    return a.flat[idx]

for filename in glob.glob(os.path.join(BVHDir, '*.bvh')):
    print "transforming: " + filename + " ..."
    parser = BvhImport(filename)
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
        if y1[index] - y2[index] > Y_THREADHOLD: #one foot is up and the other is in the floor  
                if len(stepsLfootIndexes)>0:
                    if abs(index - find_nearest(np.array(stepsLfootIndexes), index) > X_THREADHOLD): #avoid max near an existing point
                        stepsLfootIndexes.append(index)
                        print "appeend L"
                    else:
                        if y1[find_nearest(np.array(stepsLfootIndexes), index)] < y1[index]:  #check if the exiting near max is a local maximun
                            print "remove L", find_nearest(np.array(stepsLfootIndexes), index), "from: ", stepsLfootIndexes
                            stepsLfootIndexes.remove(find_nearest(np.array(stepsLfootIndexes), index))
                            print "remove L"
                            stepsLfootIndexes.append(index)
                            print "appeend L"
                else:
                    stepsLfootIndexes.append(index)
                    print "appeend L"    
    
    stepsRfootIndexes = []
    for i in range(len(maxRfootIndexes)):
        index = maxRfootIndexes[i]
        if y2[index] - y1[index] > Y_THREADHOLD: #one foot is up and the other is in the floor
                if len(stepsRfootIndexes)>0:
                    if abs(index - find_nearest(np.array(stepsRfootIndexes),index) > X_THREADHOLD): #avoid max near an existing point
                        stepsRfootIndexes.append(index)
                        print "appeend R"
                    else:
                        if y2[find_nearest(np.array(stepsRfootIndexes), index)] < y2[index]: #check if the exiting near max is a local maximun
                            print "remove R", find_nearest(np.array(stepsRfootIndexes), index), "from: ", stepsRfootIndexes, "index: ", index
                            stepsRfootIndexes.remove(find_nearest(np.array(stepsRfootIndexes), index))
                            print "remove R"
                            stepsRfootIndexes.append(index)
                            print "appeend R"
        
                else:
                    stepsRfootIndexes.append(index)
                    print "appeend R"
        

    if stepsLfootIndexes[0] < stepsRfootIndexes[0]:
        if len(stepsLfootIndexes) > 2:
            testPoint = stepsLfootIndexes[1]
            while(y1[testPoint]>y2[testPoint]):
                testPoint = testPoint + 1

            end = testPoint + 5
            print "red over green| ", "red: ", stepsLfootIndexes[0], "green: ", stepsRfootIndexes[0], "second red: ", stepsLfootIndexes[1], "end: ", end
        else:
            end = len(y1)
            print "red over green| ", "red: ", stepsLfootIndexes[0], "green: ", stepsRfootIndexes[0], "second red: -----", "end: ", end

        
    else:
        if len(stepsRfootIndexes) > 2:
            testPoint = stepsRfootIndexes[1]
            while(y2[testPoint]>y1[testPoint]):
                testPoint = testPoint + 1
            end = testPoint + 5
            print "green over red| ", "green: ", stepsRfootIndexes[0], "red: ", stepsLfootIndexes[0], "second green: ", stepsRfootIndexes[1], "end: ", end
        else:
            end = len(y2)
            print "green over red| ", "green: ", stepsRfootIndexes[0], "red: ", stepsLfootIndexes[0], "second green: -----", "end: ", end

        

    plt.plot(x1, y1,'ro')
    plt.plot(x1, y2,'g')
    plt.show()



