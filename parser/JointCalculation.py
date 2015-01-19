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

from BvhImport import BvhImport
from numpy import array
from numpy import conjugate
from numpy import angle
import math

class JointCalculation:
    def __init__(self, file):
    	self.parser = BvhImport(file)

    def angle(self,v):
        #v is a time serie, so we have to iterate in time
        for i in range(len(v)):
            if v[i].imag >=0:
    		    v[i]=angle(v[i], True) #angle second argument is for operate with degrees instead of radians
            else:
		        v[i]=180+angle(-v[i], True) #angle second argument is for operate with degrees instead of radians
        return v

    #calculates the angle in the sagital plane generated with the vectors j3 to j1 and j2 to j1 in anti clockwise
    def calculateSagital(self, joint1, joint2, joint3):
        #with points 1, 2 and 3 
        x1, y1, z1 = self.parser.getNodePositionsFromName(joint1)
        x2, y2, z2 = self.parser.getNodePositionsFromName(joint2)
        x3, y3, z3 = self.parser.getNodePositionsFromName(joint3)
        #as we are calculating in the sagital plane only z and y components are used as they describe this plane
        az1 = array(z1.values())
        az2 = array(z2.values())
        az3 = array(z3.values())
        ay1 = array(y1.values())
        ay2 = array(y2.values())
        ay3 = array(y3.values())
        #u vector, is the projection in the sagital plane of the j2 to j1 vector, 
        #complex number is usded for the sake of simplicity
        u = ay2 - ay1 + 1j*(az2 - az1)
        #v vector, is the projection in the sagital plane of the j3 to j1 vector, 
        #complex number is usded for the sake of simplicity
        v = ay3 - ay1 + 1j*(az3 - az1)
        #the angle with respect two x axis of the product of two complex numbers 
        #corresponds with the sum of the angle with x axis of each number
        r = self.angle(u*conjugate(v))
        return r.real

    #calculates the angle in the frontal plane generated with the vectors j3 to j1 and j2 to j1 in anti clockwise 
    def calculateFrontal(self, joint1, joint2, joint3):
        x1, y1, z1 = self.parser.getNodePositionsFromName(joint1)
        x2, y2, z2 = self.parser.getNodePositionsFromName(joint2)
        x3, y3, z3 = self.parser.getNodePositionsFromName(joint3)    
        az1 = array(z1.values())
        az2 = array(z2.values())
        az3 = array(z3.values())
        ax1 = array(x1.values())
        ax2 = array(x2.values())
        ax3 = array(x3.values())
        #see comments in calculateSagital
        u = (ax2 - ax1) + 1j*(az2 - az1)
        v = (ax3 - ax1) + 1j*(az3 - az1)
        r = self.angle(u*conjugate(v))
        return r.real

    #calculates the angle in the transversal plane generated with the vectors j3 to j1 and j2 to j1 in anti clockwise
    def calculateTransversal(self, joint1, joint2, joint3):
        x1, y1, z1 = self.parser.getNodePositionsFromName(joint1)
        x2, y2, z2 = self.parser.getNodePositionsFromName(joint2)
        x3, y3, z3 = self.parser.getNodePositionsFromName(joint3)    
        ay1 = array(y1.values())
        ay2 = array(y2.values())
        ay3 = array(y3.values())
        ax1 = array(x1.values())
        ax2 = array(x2.values())
        ax3 = array(x3.values())
        #see comments in calculateSagital
        u = (ax2 - ax1) + 1j*(ay2 - ay1)
        v = (ax3 - ax1) + 1j*(ay3 - ay1)
        r = self.angle(u*conjugate(v))
        return r.real