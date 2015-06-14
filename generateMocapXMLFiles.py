#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Program to convert a specific CMU mocap .bvh file to bioloid vrep models
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

from parser.JointCalculation import JointCalculation
from parser.MocapLucyMapping import MocapLucyMapping
from simulator.LoadRobotConfiguration import LoadRobotConfiguration
from configuration.LoadSystemConfiguration import LoadSystemConfiguration

from string import rstrip

import os
import glob
import ntpath
sysConf = LoadSystemConfiguration()
BVHDir = os.getcwd() + sysConf.getDirectory("CMU mocap Files")
XMLDir = os.getcwd() + sysConf.getDirectory("Transformed CMU mocap Files")
robotConfiguration = LoadRobotConfiguration()

for filename in glob.glob(os.path.join(BVHDir, '*.bvh')):
    print "transforming: " + filename + " ..."
    lucyFileConversion = MocapLucyMapping(filename, robotConfiguration)
    newFile = ntpath.basename(filename)
    newFile = rstrip(newFile[:-4]) + ".xml"
    newFile = XMLDir + newFile
    lucyFileConversion.generateFile(newFile)
    print "file: " + newFile + " generated!"
