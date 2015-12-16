#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
#
# MINA/INCO/UDELAR
#
# Damián Ferraro
# Patricia Polero
# 
# Regional Norte/UDELAR
#
# Helper library to calculate the angle in the sagital, frontal and 
# transversal plane described from three points in the cartesian space.
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

import os
import glob
import ntpath

from parser.BvhImport import BvhImport
import matplotlib.pyplot as plt
from configuration.LoadSystemConfiguration import LoadSystemConfiguration

sysConf = LoadSystemConfiguration()
BVHDir = os.getcwd() + sysConf.getDirectory("CMU mocap Files")


for filename in glob.glob(os.path.join(BVHDir, '*.bvh')):
    print "transforming: " + filename + " ..."
    parser = BvhImport(filename)
    x_,y_,z_ = parser.getNodePositionsFromName("lFoot")
    y = []
    x = []
    sizeX = 0
    for key, value in y_.iteritems():
        y.append(value)
        x.append(key)
    plt.plot(x, y, 'ro', ms=5)
    plt.show()



