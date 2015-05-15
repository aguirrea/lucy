#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Execution of the best individuals 
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

from datatypes.DTIndividualProperty             import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero, DTIndividualPropertyVanillaEvolutive
from datatypes.DTIndividualGeneticMaterial      import DTIndividualGeneticMaterial, DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix
from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
from simulator.LoadRobotConfiguration           import LoadRobotConfiguration
from Individual                                 import Individual

import os 
import glob 

propCMUDaz = DTIndividualPropertyCMUDaz()
conf = LoadSystemConfiguration()
CMUxmlDir = os.getcwd()+conf.getDirectory("Transformed CMU mocap Files")
robotConfig = LoadRobotConfiguration()
joints = robotConfig.getJointsName()

for filename in glob.glob(os.path.join(CMUxmlDir, '*.xml')):
    print 'processing individual: ' + filename
    walk = Individual(propCMUDaz, DTIndividualGeneticTimeSerieFile(filename))
    geneticMatrix = walk.getGenomeMatrix()
    for i in range(len(geneticMatrix)):
        for j in joints:
            print "pose number: " + str(i) + " joint: " + j + " value: " + str(geneticMatrix[i][walk.getJointMatrixIDFromName(j)])

    