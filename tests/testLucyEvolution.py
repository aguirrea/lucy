#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Execution of individuals resulted from the evolution experiment
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

import glob
import os
import sys

from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
from datatypes.DTIndividualGeneticMaterial      import DTIndividualGeneticTimeSerieFile, \
    DTIndividualGeneticTimeSerieFileWalk
from datatypes.DTIndividualProperty             import DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero, DTIndividualPropertyVanillaEvolutive, DTIndividualPropertyPhysicalBioloid, DTIndividualPropertyVanillaEvolutiveNoAvoid

from Individual                                 import Individual

propCMUDaz = DTIndividualPropertyCMUDaz()
propVanilla = DTIndividualPropertyVanilla()
balieroProp = DTIndividualPropertyBaliero()
physicalProp = DTIndividualPropertyPhysicalBioloid()
geneticVanillaProp = DTIndividualPropertyVanillaEvolutive()
geneticVanillaPropNothingToAvoid = DTIndividualPropertyVanillaEvolutiveNoAvoid()

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

        if int(conf.getProperty("Concatenate walk cylcles?")):
            walkEmbryo = DTIndividualGeneticTimeSerieFileWalk(os.getcwd()+"/"+filename)
        else:
            walkEmbryo = DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename)
        #walk = Individual(geneticVanillaPropNothingToAvoid, DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename)) #For Reda Al-Bahrani work compability
        precycleFile = os.getcwd()+"/mocap/cmu_mocap/xml/util/walk_precycle.xml"
        preCycleEmbryo = DTIndividualGeneticTimeSerieFile(precycleFile)
        preCycleEmbryo.concatenate(walkEmbryo)
        walkEmbryo = preCycleEmbryo
        walk = Individual(geneticVanillaProp, walkEmbryo)

    else:
        walk = Individual(physicalProp, DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename))
        #TODO add support for walking cycle
    return walk

'''walk = Individual(geneticVanillaProp, DTIndividualGeneticMatrix()) #dummy individual to initialise the simulator and enable the time step configuration
walk.execute()
print "please set the proper time step in vrep"
time.sleep(5)'''
if arguments > 1:
    files = sys.argv[1:]
    for filename in files:
        print 'executing individual: ' + filename
        walk = createIndividual(filename)
        walk.execute()
else:
    for filename in glob.glob(os.path.join(geneticPoolDir, '*.xml')):
        print 'executing individual: ' + filename
        walk = Individual(geneticVanillaProp, DTIndividualGeneticTimeSerieFile(filename))
        walk.execute()

