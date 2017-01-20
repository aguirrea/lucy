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
import time

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

        precycleFile = os.getcwd()+"/mocap/cmu_mocap/xml/util/walk_precycle.xml"
        preCycleEmbryo = DTIndividualGeneticTimeSerieFile(precycleFile)
        preCycleLength = preCycleEmbryo.getLength()

        if int(conf.getProperty("Concatenate walk cycles?")):
            walkEmbryo = DTIndividualGeneticTimeSerieFileWalk(os.getcwd()+"/"+filename)
            embryoCycleLength = (walkEmbryo.getLength() - preCycleLength) / int(conf.getProperty("Concatenate walk cycles?"))
        else:
            walkEmbryo = DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename)
            embryoCycleLength = walkEmbryo.getLength() - preCycleLength
        #walk = Individual(geneticVanillaPropNothingToAvoid, DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename)) #For Reda Al-Bahrani work compability


        if int(conf.getProperty("concatenate external cycle file?")):
            externalFirstCycleFile = os.getcwd() + conf.getFile("external cycle file")
            externalFirstCycle = DTIndividualGeneticTimeSerieFile(externalFirstCycleFile)
            preCycleEmbryo.concatenate(externalFirstCycle)
            preCycleLength = preCycleEmbryo.getLength()
            embryoCycleLength = embryoCycleLength - externalFirstCycle.getLength()


        preCycleEmbryo.concatenate(walkEmbryo)
        walkEmbryo = preCycleEmbryo
        walk = Individual(geneticVanillaProp, walkEmbryo)
        walk.setPrecycleLength(preCycleLength)
        walk.setCycleLength(embryoCycleLength)

    else:
        #TODO physical stage
        #TODO restructure the precycle for the case of physical and simulated
        walkEmbryo = DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename)
        precycleFile = os.getcwd()+"/mocap/cmu_mocap/xml/util/walk_precycle.xml"
        preCycleEmbryo = DTIndividualGeneticTimeSerieFile(precycleFile)
        preCycleEmbryo.concatenate(walkEmbryo)

        if int(conf.getProperty("concatenate external cycle file?")):
            precycleFile = os.getcwd() + conf.getFile("external cycle file")
            firstCycle = DTIndividualGeneticTimeSerieFile(precycleFile)
            preCycleEmbryo.concatenate(firstCycle)

        walkEmbryo = preCycleEmbryo
        walk = Individual(physicalProp, walkEmbryo)
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
        print "generating individual"
        time1 = time.time()
        walk = createIndividual(filename)
        print "creation time: ", time.time() - time1
        time1 = time.time()
        print "executing individual!"
        walk.execute()
        print "execution time: ", time.time() - time1
        print "individual executed"
else:
    for filename in glob.glob(os.path.join(geneticPoolDir, '*.xml')):
        print 'executing individual: ' + filename
        print "generating individual"
        time1 = time.time()
        walk = Individual(geneticVanillaProp, DTIndividualGeneticTimeSerieFile(filename))
        print "creation time: ", time.time() - time1
        time1 = time.time()
        print "executing individual!"
        walk.execute()
        print "execution time: ", time.time() - time1
        print "individual executed"


