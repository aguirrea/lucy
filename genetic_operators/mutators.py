#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Lucy mutator operator based on pyevolve G2DListMutatorRealGaussian modified 
# using UnivariateSpline to smmoth the poses near the mutation point
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


from random import randint as rand_randint, gauss as rand_gauss
from random import random as rand_random

from datatypes.DTIndividualProperty import DTIndividualPropertyVanillaEvolutive

import configuration.constants as sysConstants
from genetic_operators.DTGenomeFunctions import DTGenomeFunctions
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

CDefG2DListMutRealMU = 0
CDefG2DListMutRealSIGMA = sysConstants.MUTATION_SIGMA

CDefRangeMin = 0
CDefRangeMax = 100


def chromosomeToLucyGeneticMatrix(chromosome): #TODO encapsulate this in a helper class
    geneticMatrix = [[chromosome[i][j] for j in xrange(chromosome.getWidth())] for i in xrange(chromosome.getHeight())]
    return geneticMatrix
    #TODO remove '-1' values

def randomFlipCoin(p):
   """ Returns True with the *p* probability. If the *p* is 1.0,
   the function will always return True, or if is 0.0, the
   function will return always False.

   Example:
      >>> randomFlipCoin(1.0)
      True

   :param p: probability, between 0.0 and 1.0
   :rtype: True or False

   extracted from pyevolve
   """
   if p == 1.0: return True
   if p == 0.0: return False

   return True if rand_random() <= p else False

def G2DListMutatorRealGaussianSpline(genome, **args):
    """ A gaussian mutator for G2DList of Real with UnivariateSpline for smoothing
    Accepts the *rangemin* and *rangemax* genome parameters, both optional. Also
    accepts the parameter *gauss_mu* and the *gauss_sigma* which respectively
    represents the mean and the std. dev. of the random distribution. """

    dtgenome = DTGenomeFunctions()
    pmut = args["pmut"]
    if pmut <= 0.0: return 0
    width = dtgenome.getIndividualFrameLength(genome)
    height = dtgenome.getIndividualLength(genome)
    elements = height * width

    mutations = pmut * elements

    mu = genome.getParam("gauss_mu")
    sigma = genome.getParam("gauss_sigma")

    prop = DTIndividualPropertyVanillaEvolutive()
    robotConfig = LoadRobotConfiguration()
    robotJoints = robotConfig.getJointsName()

    validIDs = []
    jointIndex = 0
    for joint in robotJoints:
        if not prop.avoidJoint(joint):
            validIDs.append(jointIndex)
            jointIndex += 1

    if mu is None:
        mu = CDefG2DListMutRealMU

    if sigma is None:
        sigma = CDefG2DListMutRealSIGMA

    if mutations < 1.0:
        mutations = 0

        for i in xrange(genome.getHeight()):
            for j in xrange(genome.getWidth()):
                if genome[i][j] != sysConstants.JOINT_SENTINEL and j in validIDs and randomFlipCoin(pmut):
                    offset = rand_gauss(mu, sigma)
                    print "OFFSET: ", offset, "joint: ", j, "frame: ", i
                    final_value = genome[i][j] + offset

                    final_value = min(final_value, genome.getParam("rangemax", CDefRangeMax))
                    final_value = max(final_value, genome.getParam("rangemin", CDefRangeMin))

                    genome.setItem(i, j, final_value)
                    dtgenome.interpolate(genome, j, i, offset)
                    ##pca.poseInterpolationWithPCA(chromosomeToLucyGeneticMatrix(genome), j)
                    mutations += 1
    else:
        for it in xrange(int(round(mutations))):
            foundFrameDifferentThanSentinel = False
            validIDFound = False
            while not foundFrameDifferentThanSentinel and not validIDFound:
                which_x = rand_randint(0, genome.getWidth() - 1)  # joint to mutate
                if which_x in validIDs: #only mutate not avoid ids
                    validIDFound = True
                    which_y = rand_randint(0, genome.getHeight() - 1)  # pose to mutate
                    if genome[which_y][which_x] != sysConstants.JOINT_SENTINEL:
                        foundFrameDifferentThanSentinel = True
                        offset = rand_gauss(mu, sigma)
                        final_value = genome[which_y][which_x] + offset
                        #print "OFFSET: ", offset, "joint: ", which_x, "frame: ", which_y

                        # to be sure that the value is less than the rangemax and more than the rangemin value
                        final_value = min(final_value, genome.getParam("rangemax", CDefRangeMax))
                        final_value = max(final_value, genome.getParam("rangemin", CDefRangeMin))

                        # valueBeforeMutation = genome[which_y][which_x]
                        genome.setItem(which_y, which_x, final_value)
                        dtgenome.interpolate(genome, which_x, which_y)
                        ##pca.poseInterpolationWithPCA(chromosomeToLucyGeneticMatrix(genome), which_x)


    return int(mutations)
