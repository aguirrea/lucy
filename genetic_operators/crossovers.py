#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Lucy crossover-operators
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

from random import randint as rand_randint
import sys
import configuration.constants as sysConstants
from genetic_operators.DTGenomeFunctions import DTGenomeFunctions

INFINITE_DISTANCE = sys.maxint

def G2DListCrossoverSingleNearHPoint(genome, **args):
    """ 
    The crossover of G2DList, Single Horizontal Point using the near most similar gen  

    mom = X1X2X3X4X5X6X7X8X9X10X11X12
    dad = Y1Y2Y3Y4Y5Y6Y7Y8Y9Y10
    
    mom_rand_cut = X6
    dad_minimal_diff_position = Y3 

    sister = X1X2X3X4X5Y3Y4Y5Y6Y7Y9Y10
    brother = Y1Y2X6X7X8X9X10X11X12

    """
    dtgenome = DTGenomeFunctions()
    sister = None
    brother = None
    gMom = args["mom"]
    gDad = args["dad"]
    gDadLength = dtgenome.getIndividualLength(gDad)
    gMomLenght = dtgenome.getIndividualLength(gMom)
    print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    print "genome dad length: ", gDadLength
    print "genome mom length: ", gMomLenght
    MINIMAL_DIFF = INFINITE_DISTANCE
    MINIMAL_DIFF_POSITION = 0
    DIFFERENCE_BETWEEN_POSES_THREADHOLD = 15
    MINIMAL_CROSSOVER_POINT = 0

    if gMomLenght > MINIMAL_CROSSOVER_POINT :
        cut = rand_randint(MINIMAL_CROSSOVER_POINT, gMomLenght - 1)
    else: #trying to preserve the walk cycle unit we use "restrictions on the cross"
        cut = 0

    frame1 = gMom[cut]

    for position in xrange(gDadLength):
        frameDiff = dtgenome.diff(frame1, gDad[position])
        if frameDiff < MINIMAL_DIFF:
            MINIMAL_DIFF = frameDiff
            MINIMAL_DIFF_POSITION = position

    if MINIMAL_DIFF < DIFFERENCE_BETWEEN_POSES_THREADHOLD: #trying to preserve the walk cycle unit we use "restrictions on the cross"
        print "difference between poses: ", MINIMAL_DIFF, "in position: ", MINIMAL_DIFF_POSITION

        #TODO comment this, only here for debugging
        gSisterLength = cut + gDadLength - MINIMAL_DIFF_POSITION
        if gSisterLength > sysConstants.GENOMA_MAX_LENGTH:
            gSisterLength = sysConstants.GENOMA_MAX_LENGTH - 1
            print "************warning gSisterLength > sysConstants.GENOMA_MAX_LENGTH**************"

        #TODO comment this, only here for debugging
        gBrotherLength = MINIMAL_DIFF_POSITION + gMomLenght - cut
        if gBrotherLength > sysConstants.GENOMA_MAX_LENGTH:
            gBrotherLength = sysConstants.GENOMA_MAX_LENGTH - 1
            print "************warning gBrotherLength > sysConstants.GENOMA_MAX_LENGTH*************"

        if args["count"] >= 1:
            sister = gMom.clone()
            sister.resetStats()
            for i in xrange(MINIMAL_DIFF_POSITION, gDadLength):
                sisterIndex = cut+i-MINIMAL_DIFF_POSITION
                if sisterIndex < sysConstants.GENOMA_MAX_LENGTH -1:
                    sister[sisterIndex][:] = gDad[i][:]
                else:
                    for joint in xrange(sister.getWidth()):
                      sister[sisterIndex][joint] = sysConstants.JOINT_SENTINEL
                    break

            print "gSisterLength: ", gSisterLength

        if args["count"] == 2:
            brother = gDad.clone()
            brother.resetStats()
            for i in xrange(MINIMAL_DIFF_POSITION, MINIMAL_DIFF_POSITION + gMomLenght - cut):
                if i < sysConstants.GENOMA_MAX_LENGTH -1:
                    brother[i][:] = gMom[cut+i-MINIMAL_DIFF_POSITION][:]
                else:
                    for joint in xrange(brother.getWidth()): #TODO usar método para obtener el frame length de dtgenomefunctions
                       brother[i][joint] = sysConstants.JOINT_SENTINEL
                    break

            print "gBrotherLength: ", gBrotherLength

        print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"

        '''else:
            print "***dad?: ", gDad[0]
            print "***mom?: ", gMom[0]
            print "***dad?: ", gDad[1]
            print "***mom?: ", gMom[1]
        print "counts args: ", args["count"]'''
    else:
        sister = gMom.clone()
        brother = gDad.clone()
        print "difference between poses less than DIFFERENCE_BETWEEN_POSES_THREADHOLD not found"

    return (sister, brother)

