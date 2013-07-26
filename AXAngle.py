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

class AXAngle:
    def __init__(self, value):
        self.angle = value
        
    def getValue(self):
        return(self.angle)
    
    def setValue(self,value):
        self.angle = value
        
    def toDegrees(self):
        return((self.angle*300)/1023)
    
    def toRadians(self):
        return(math.radians(self.toDegrees()))
        

angle = AXAngle(1023)
angle.setValue(1023)
value = angle.getValue()

print value
print (angle.toDegrees())
print (angle.toRadians())