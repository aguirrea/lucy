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

from datatypes.DTIndividualProperty import DTIndividualPropertyVanillaEvolutive

import configuration.constants as sysConstants
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

INFINITE_DISTANCE = sys.maxint
SPLINE_SMOOTHING_FACTOR_INTERPOLATION = 5
SPLINE_SMOOTHING_FACTOR_SPLINE = 5
SMOOTHING_WINDOW = 20 #must be multiple of 4

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
            if not prop.diffAvoidJoint(joint):
                if frame1[jointIndex] == sysConstants.JOINT_SENTINEL or frame2[jointIndex] == sysConstants.JOINT_SENTINEL:
                    diff = INFINITE_DISTANCE
                    break
                else:
                    jointDiff = (frame1[jointIndex] - frame2[jointIndex]) ** 2
                    diff += abs(jointDiff)
                    #print "joint: ", joint, "diff: ", jointDiff, "frame1[jointIndex]", frame1[jointIndex], "frame2[jointIndex]", frame2[jointIndex]
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
                jointDiff = abs(frame1[jointIndex] - frame2[jointIndex])
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
        minimunInterpolationNeighborhoodSize = interpolationPointsQty / 4

        if which_y - interpolationPointsQty / 2 < 0:
            interpolationPointsQty -= abs(which_y - which_y_InterpolationNeighborhood) * 2
            which_y_InterpolationNeighborhood = interpolationPointsQty / 2

        elif which_y + interpolationPointsQty / 2 > genome.getHeight() - 1:
            interpolationPointsQty -= (which_y + which_y_InterpolationNeighborhood - (genome.getHeight() - 1)) * 2
            which_y_InterpolationNeighborhood = interpolationPointsQty / 2

        if which_y_InterpolationNeighborhood >= minimunInterpolationNeighborhoodSize:
            x = np.ndarray(interpolationPointsQty)
            y = np.ndarray(interpolationPointsQty)

            for k in xrange(interpolationPointsQty):
                poseToSmooth = which_y - which_y_InterpolationNeighborhood + k
                x[k] = poseToSmooth
                y[k] = genome[poseToSmooth][which_x]

            spl = UnivariateSpline(x, y)
            spl.set_smoothing_factor(1/SPLINE_SMOOTHING_FACTOR_SPLINE)

            for k in xrange(interpolationPointsQty):
                if y[k] != sysConstants.JOINT_SENTINEL:
                    newValue = spl(int(x[k]))
                    genome.setItem(int(x[k]), which_x, newValue)

    #/____A____/____B_____ /which_y/____C_____/_____D____/   B + C are the interpolationWindow, A + B + C + D  are the
    # interpolationPointsQty. using A and D as fixed points, interpolate B and C
    def interpolate(self, genome, which_x, which_y, wich_y_is_fixed_data=0):
        interpolationPointsQty = SMOOTHING_WINDOW
        which_y_InterpolationNeighborhood = interpolationPointsQty / 2
        minimunInterpolationNeighborhoodSize = interpolationPointsQty / 4
        array_size = 0

        if which_y - which_y_InterpolationNeighborhood < 0:
            interpolationPointsQty -= abs(which_y - which_y_InterpolationNeighborhood) * 2
            which_y_InterpolationNeighborhood = interpolationPointsQty / 2

        elif which_y + interpolationPointsQty / 2 > genome.getHeight() - 1:
            interpolationPointsQty -= (which_y + which_y_InterpolationNeighborhood - (genome.getHeight() - 1)) * 2
            which_y_InterpolationNeighborhood = interpolationPointsQty / 2

        interpolationWindowRadius = interpolationPointsQty / 4


        if which_y_InterpolationNeighborhood >= minimunInterpolationNeighborhoodSize:
            array_size = interpolationPointsQty - interpolationWindowRadius * 2
            if wich_y_is_fixed_data:
                array_size += 1

            x = np.ndarray(array_size)
            y = np.ndarray(array_size)

            splineIndexCounter = 0
            for k in xrange(interpolationPointsQty):
                poseToSmooth = which_y - which_y_InterpolationNeighborhood + k
                if poseToSmooth <= which_y - interpolationWindowRadius or poseToSmooth > which_y + interpolationWindowRadius:
                    x[splineIndexCounter] = poseToSmooth
                    y[splineIndexCounter] = genome[poseToSmooth][which_x]
                    splineIndexCounter += 1

            if wich_y_is_fixed_data:
                x[splineIndexCounter] = which_y
                y[splineIndexCounter] = genome[which_y][which_x]
                splineIndexCounter += 1

            x_order = np.argsort(x)
            spl = UnivariateSpline(x_order, y)
            spl.set_smoothing_factor(1/SPLINE_SMOOTHING_FACTOR_INTERPOLATION)
            for k in xrange(interpolationPointsQty):
                iter = which_y - which_y_InterpolationNeighborhood + k
                if genome[iter][which_x] != sysConstants.JOINT_SENTINEL:
                    if iter > which_y - interpolationWindowRadius and iter <= which_y + interpolationWindowRadius:
                        if wich_y_is_fixed_data: #if fixed data do not change the which_y point
                            if iter != which_y:
                                newValue = spl(iter)
                                genome.setItem(iter, which_x, newValue)
                        else:
                            newValue = spl(iter)
                            genome.setItem(iter, which_x, newValue)
