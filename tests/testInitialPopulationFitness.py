#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Test case for evaluate the fitness of the initial population
# using the CMU database located in /mocap/cmu_mocap
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


from simulator.Lucy                             import SimulatedLucy
from simulator.AXAngle                          import AXAngle
from parser.LoadPoses                           import LoadPoses
from datatypes.DTIndividualProperty             import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero, DTIndividualPropertyVanillaEvolutive, DTIndividualPropertyPhysicalBioloid
from datatypes.DTIndividualGeneticMaterial      import DTIndividualGeneticMaterial, DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix
from Pose                                       import Pose
from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
from simulator.LoadRobotConfiguration           import LoadRobotConfiguration
from Individual                                 import Individual

import os
import glob
import time
import sys

propCMUDaz = DTIndividualPropertyCMUDaz()
propVanilla = DTIndividualPropertyVanilla()
balieroProp = DTIndividualPropertyBaliero()
physicalProp = DTIndividualPropertyPhysicalBioloid()
geneticVanillaProp = DTIndividualPropertyVanillaEvolutive()

conf = LoadSystemConfiguration()

CMUxmlDir = os.getcwd()+conf.getDirectory("Transformed CMU mocap Files")


for filename in glob.glob(os.path.join(CMUxmlDir, '*.xml')):
    walk = Individual(propCMUDaz, DTIndividualGeneticTimeSerieFile(filename))
    fitness = walk.execute()
    length = walk.getLength()
    walk.persist("/tmp/"+str(fitness)+".xml")
    print "individual: ", filename, "fitness: ", fitness, "length: ", length
