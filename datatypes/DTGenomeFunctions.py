#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Datatype with helper functions for the genetic individual representation.
# helps to visualize the G2DList genoma representation into a one dimensional representation
# with a gen corresponding to a mocap pose.
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
import sys
from scipy.interpolate import UnivariateSpline

import configuration.constants as sysConstants
from datatypes.DTIndividualProperty import DTIndividualPropertyVanillaEvolutive
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

INFINITE_DISTANCE = sys.maxint
SPLINE_SMOOTHING_FACTOR = 5
SMOOTHING_WINDOW = 20

class DTGenomeFunctions(object):
    def __init__(self):
        self.robotConfig = LoadRobotConfiguration()

    # returns the distance between two frame poses, if one of the poses is a sentinel pose the distance is INFINITE_DISTANCE
    # uses square power to make visible a change in one of the joints
    def euclideanDiff(self, frame1, frame2):
        diff = 0
        prop = DTIndividualPropertyVanillaEvolutive()
        robotJoints = self.robotConfig.getJointsName()
        jointIndex = 0
        for joint in robotJoints:
            if not prop.avoidJoint(joint):
                if frame1[jointIndex] == sysConstants.JOINT_SENTINEL or frame2[
                    jointIndex] == sysConstants.JOINT_SENTINEL:
                    diff = INFINITE_DISTANCE
                    break
                else:
                    jointDiff = (frame1[jointIndex] - frame2[jointIndex]) ** 2
                    diff += jointDiff
            jointIndex = jointIndex + 1
        return diff

    # returns the distance between two frame poses, if one of the poses is a sentinel pose the distance is INFINITE_DISTANCE
    def rawDiff(self, frame1, frame2):
        diff = 0
        prop = DTIndividualPropertyVanillaEvolutive()
        robotJoints = self.robotConfig.getJointsName()
        jointIndex = 0
        for joint in robotJoints:
            if not prop.avoidJoint(joint):
                jointDiff = frame1[jointIndex] - frame2[jointIndex]
                diff += jointDiff
            jointIndex = jointIndex + 1
        return diff

    # compares every joint of a frame with the JOINT_SENTINEL value, and return True if every joint has a value equal to it
    def equalSentinelFrame(self, frame):
        sentinelFramePresent = True
        for jointValue in frame:
            sentinelFramePresent = sentinelFramePresent and jointValue == sysConstants.JOINT_SENTINEL
        return sentinelFramePresent

    # returns the length of a genome, as the number of poses it contains until the JOINT_SENTINEL value is present in all the joints of a pose
    def getIndividualLength(self, genome):
        iter = 0
        genomaHeight = genome.getHeight()
        while iter < genomaHeight and not self.equalSentinelFrame(genome[iter]):
            iter = iter + 1
        length = iter
        return length

    def getIndividualFrameLength(self, genome):
        return genome.getWidth()

    def smooth(self, genome, which_x, which_y):
        interpolationPointsQty = SMOOTHING_WINDOW
        which_y_InterpolationNeighborhood = interpolationPointsQty / 2
        minimunInterpolationNeighborhoodSize = 4

        if which_y - interpolationPointsQty / 2 < 0:
            interpolationPointsQty -= abs(which_y - which_y_InterpolationNeighborhood) * 2
            which_y_InterpolationNeighborhood = interpolationPointsQty / 2

        elif which_y + interpolationPointsQty / 2 > genome.getHeight() - 1:
            interpolationPointsQty -= (which_y + which_y_InterpolationNeighborhood - (genome.getHeight() - 1)) * 2
            which_y_InterpolationNeighborhood = interpolationPointsQty / 2

        if which_y_InterpolationNeighborhood > minimunInterpolationNeighborhoodSize:
            x = np.ndarray(interpolationPointsQty)
            y = np.ndarray(interpolationPointsQty)

            for k in xrange(interpolationPointsQty):
                poseToSmooth = which_y - which_y_InterpolationNeighborhood + k
                x[k] = poseToSmooth
                y[k] = genome[poseToSmooth][which_x]

            spl = UnivariateSpline(x, y, s=SPLINE_SMOOTHING_FACTOR)

            for k in xrange(interpolationPointsQty):
                # print "before ", genome[int(x[k])][which_x], "now ", spl(int(x[k])), "k ", k, "diff ", genome[int(x[k])][which_x]-spl(int(x[k])), "offset ", offset
                # if x[k] == which_x:
                #    print "no mutation"
                # elif y[k]!= sysConstants.JOINT_SENTINEL:
                if y[k] != sysConstants.JOINT_SENTINEL:
                    newValue = spl(int(x[k]))
                    genome.setItem(int(x[k]), which_x, newValue)
                    ##print "mutating value: ", y[k], "to value: ", newValue, "difference: ", abs(y[k]-newValue)
                    ##else:
                    ##print "oh no!, we have a problem in mutation operator .. :S"

    def interpolate(self, genome, which_x, which_y):
        interpolationPointsQty = SMOOTHING_WINDOW
        which_y_InterpolationNeighborhood = interpolationPointsQty / 2
        minimunInterpolationNeighborhoodSize = 4

        if which_y - interpolationPointsQty / 2 < 0:
            interpolationPointsQty -= abs(which_y - which_y_InterpolationNeighborhood) * 2
            which_y_InterpolationNeighborhood = interpolationPointsQty / 2

        elif which_y + interpolationPointsQty / 2 > genome.getHeight() - 1:
            interpolationPointsQty -= (which_y + which_y_InterpolationNeighborhood - (genome.getHeight() - 1)) * 2
            which_y_InterpolationNeighborhood = interpolationPointsQty / 2

        interpolationWindow = interpolationPointsQty / 5

        if which_y_InterpolationNeighborhood > minimunInterpolationNeighborhoodSize:
            x = np.ndarray(interpolationPointsQty - interpolationWindow * 2)
            y = np.ndarray(interpolationPointsQty - interpolationWindow * 2)

            splineIndexCounter = 0
            for k in xrange(interpolationPointsQty):
                poseToSmooth = which_y - which_y_InterpolationNeighborhood + k
                if poseToSmooth <= which_y - interpolationWindow or poseToSmooth > which_y + interpolationWindow:
                    x[splineIndexCounter] = poseToSmooth
                    y[splineIndexCounter] = genome[poseToSmooth][which_x]
                    splineIndexCounter += 1

            spl = UnivariateSpline(x, y, s=SPLINE_SMOOTHING_FACTOR)

            for k in xrange(interpolationPointsQty):
                iter = which_y - which_y_InterpolationNeighborhood + k
                if genome[iter][which_x] != sysConstants.JOINT_SENTINEL:
                    newValue = spl(iter)
                    genome.setItem(iter, which_x, newValue)
