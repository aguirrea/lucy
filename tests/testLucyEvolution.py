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

from simulator.SimLucy                          import SimLucy
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
GAwalkDir = os.getcwd()+conf.getDirectory("GAWalk Files")
UIBLHDir = os.getcwd()+conf.getDirectory("UIBLH mocap Files")
BalieroDir = os.getcwd()+conf.getDirectory("Baliero transformed walk Files")
ADHOCDir = os.getcwd()+conf.getDirectory("ADHOC Files")
geneticPoolDir = os.pardir+conf.getDirectory("Genetic Pool")

arguments = len(sys.argv)

def createIndividual(filename):
    if int(conf.getProperty("Lucy simulated?"))==1:
        walk = Individual(geneticVanillaProp, DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename))
    else:
        walk = Individual(physicalProp, DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename))    
    return walk

if arguments > 1:
    files = sys.argv[1:]
    for filename in files:
        print 'executing individual: ' + filename
        walk = createIndividual(filename)
else:
    for filename in glob.glob(os.path.join(geneticPoolDir, '*.xml')):
        print 'executing individual: ' + filename
        walk = Individual(geneticVanillaProp, DTIndividualGeneticTimeSerieFile(filename))

walk.execute()

