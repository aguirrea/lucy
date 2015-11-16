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

from parser.LoadPoses import LoadPoses
from simulator.LoadRobotConfiguration import LoadRobotConfiguration


class DTIndividualGeneticMaterial(object):
    def __init__(self):
        self.geneticMatrix=[]

    def getGeneticMatrix(self):
        return self.geneticMatrix

class DTIndividualGeneticTimeSerieFile(DTIndividualGeneticMaterial):
    
    def __init__(self, geneticMaterial):
        DTIndividualGeneticMaterial.__init__(self)
        lp = LoadPoses(geneticMaterial)
        robotConfig = LoadRobotConfiguration()
        poseSize = lp.getFrameQty()
        self.geneticMatrix = [[lp.getPose(i).getValue(j) for j in robotConfig.getJointsName()] for i in xrange(poseSize)] #debería pedir solo los joints implementados

class DTIndividualGeneticMatrix(DTIndividualGeneticMaterial):

    def __init__(self, geneticMaterial=[[0 for j in xrange(18)] for i in xrange(1)]):
        DTIndividualGeneticMaterial.__init__(self)
        self.geneticMatrix = geneticMaterial




