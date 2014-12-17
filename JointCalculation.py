#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# Damián Ferraro
# Patricia Polero
#
# MINA/INCO/UDELAR
# 
# Joint value calculations for the bioloid model from mocap model
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
        for i in range(len(v)):
            if v[i].imag >=0:
    		    v[i]=angle(v[i], True) #angle second argument is for operate with degrees instead of radians
            else:
		        v[i]=180-angle(-v[i], True) #angle second argument is for operate with degrees instead of radians
        return v

    #from joint3 to joint2 with axis in joint1 anti clockwise
    def calculateSagital(self, joint1, joint2, joint3):
        x1, y1, z1 = self.parser.getNodePositionsFromName(joint1)
        x2, y2, z2 = self.parser.getNodePositionsFromName(joint2)
        x3, y3, z3 = self.parser.getNodePositionsFromName(joint3)
        az1 = array(z1.values())
        az2 = array(z2.values())
        az3 = array(z3.values())
        ay1 = array(y1.values())
        ay2 = array(y2.values())
        ay3 = array(y3.values())
        u = ay2 - ay1 + 1j*(az2 - az1)
        v = ay3 - ay1 + 1j*(az3 - az1)
        r = self.angle(u*conjugate(v))
        return r.real
    #from joint3 to joint2 with axis in joint1 anti clockwise
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
        u = (ax2 - ax1) + 1j*(az2 - az1)
        v = (ax3 - ax1) + 1j*(az3 - az1)
        r = self.angle(u*conjugate(v))
        return r.real

 	#from joint3 to joint2 with axis in joint1 anti clockwise
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
        u = (ax2 - ax1) + 1j*(ay2 - ay1)
        v = (ax3 - ax1) + 1j*(ay3 - ay1)
        r = self.angle(u*conjugate(v))
        return r.real

        
jc = JointCalculation("02_02.bvh")
#rElbowYaw = jc.calculateSagital("rForeArm", "rShldr", "rHand") #validado

#print "********************************************************************************************"
rShoulderYaw = jc.calculateTransversal("rCollar","chest", "rShldr") #validado
rShoulderPitch = jc.calculateSagital("rShldr", "rForeArm", "hip") #validado 
rHipYaw = jc.calculateTransversal("rThigh", "hip", "rShin") #validado 
#rHipRoll = jc.calculateFrontal("rThigh","hip" , "rShin")  #validado 
#rHipPitch = jc.calculateSagital("rThigh", "abdomen", "rShin") #validado
#rKneePitch = jc.calculateSagital("rShin", "rThigh","rFoot") #validado
#rAnkle = jc.calculateSagital("rFoot", "rShin", "End Site") #validado
