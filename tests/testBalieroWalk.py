#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Execution of individuals resulted from the Baliero and Pias work
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
from datatypes.DTIndividualGeneticMaterial      import DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix
from datatypes.DTIndividualProperty             import DTIndividualPropertyBaliero, DTIndividualPropertyPhysicalBioloid

from Individual                                 import Individual

balieroProp = DTIndividualPropertyBaliero()
physicalProp = DTIndividualPropertyPhysicalBioloid()


conf = LoadSystemConfiguration()

BalieroDir = os.getcwd()+conf.getDirectory("Baliero transformed walk Files")

arguments = len(sys.argv)

def createIndividual(filename):
    if int(conf.getProperty("Lucy simulated?"))==1:
        walk = Individual(balieroProp, DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename))
    else:
        walk = Individual(physicalProp, DTIndividualGeneticTimeSerieFile(os.getcwd()+"/"+filename))
    return walk

walk = Individual(balieroProp, DTIndividualGeneticMatrix()) #dummy individual to initialise the simulator and enable the time step configuration
walk.execute()
print "please set the proper time step in vrep"
time.sleep(5)
if arguments > 1:
    files = sys.argv[1:]
    for filename in files:
        print 'executing individual: ' + filename
        walk = createIndividual(filename)
        walk.execute()
else:
    for filename in glob.glob(os.path.join(BalieroDir, '*.xml')):
        print 'executing individual: ' + filename
        walk = createIndividual(filename)
        walk.execute()

