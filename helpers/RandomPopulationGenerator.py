#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Generates a initial population of random individuals for the GA
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

import glob, os, shutil
import time

from tests.configuration.LoadSystemConfiguration import LoadSystemConfiguration
from helpers.RandomIndividualGenerator import RandomIndividualGenerator

conf = LoadSystemConfiguration()
geneticPoolDir = os.getcwd() + "/.." + conf.getDirectory("Random Individual Files")
#geneticPoolDirTmp = os.path.join(geneticPoolDir, "tmp")
geneticPoolDirTmp = geneticPoolDir + "tmp/"
print geneticPoolDirTmp
initialPopulationSize = int(conf.getProperty("Population size"))


initialPopulationIndividualFiles = glob.iglob(os.path.join(geneticPoolDir, "*.xml"))
for file in initialPopulationIndividualFiles:
    if os.path.isfile(file):
        shutil.move(file, geneticPoolDirTmp)

for i in range(initialPopulationSize):
    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = geneticPoolDir + "random-" + str(i) + "-" + timestr + ".xml"
    print "creating file: ", filename
    randInd = RandomIndividualGenerator(filename)
    randInd.generateFile()
