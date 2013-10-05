#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# basic ax12 actuator control
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

import math
from math import radians

MODEL_ZERO_VALUE = 150

class AXAngle:
    def __init__(self, angle=0, rotation=1):
        self.angle = angle
        self.rotation = rotation
        
    def getValue(self):
        return(self.angle)
    
    def setValue(self,value):
        self.angle = value

    # value is 1 for a clockwise rotation 0 anticlockwise
    def setRotation(self, value):
        self.rotation = value
        
    def getRotation(self):
        return (self.rotation)

    def toDegrees(self):
        return((self.angle*300)/1023)
    
    def toRadians(self):
        return(math.radians(self.toDegrees()))
    
    def toVrep(self):
        if (self.getRotation() == 1):
            return ((self.toDegrees()-MODEL_ZERO_VALUE)/float(60))    
        else:
            return (-1*(self.toDegrees()-MODEL_ZERO_VALUE)/float(60))                


#angle = AXAngle(0)
#for i in range(1023) :
#    angle.setValue(i)
#    value = angle.toVrep()
#    print value

#print value
#print (angle.toDegrees())
#print (angle.toRadians())
#print (angle.toVrep())
