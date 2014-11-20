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
    		    v[i]=angle(v[i])
            else:
		        v[i]=(math.pi)-angle(-v[i])
        return v
    #from joint3 to joint2 with axis in joint1 anti clockwise
    def calculate(self, joint1, joint2, joint3):
        x1, y1, z1 = self.parser.getNodePositionsFromName(joint1)
        x2, y2, z2 = self.parser.getNodePositionsFromName(joint2)
        x3, y3, z3 = self.parser.getNodePositionsFromName(joint3)
        ax1 = array(x1.values())
        ax2 = array(x2.values())
        ax3 = array(x3.values())
        ay1 = array(y1.values())
        ay2 = array(y2.values())
        ay3 = array(y3.values())
        u = ax2 - ax1 + 1j*(ay2 - ay1)
        v = ax3 - ax1 + 1j*(ay3 - ay1)
        conj=conjugate(v)
        r = self.angle(u*conjugate(v))*(180/math.pi)
        return r.real
        
jc = JointCalculation("02_02.bvh")
rElbowYaw = jc.calculate("rForeArm", "rShldr", "rHand")
print rElbowYaw
print rElbowYaw.min()
print rElbowYaw.max()

#print "********************************************************************************************"
#rShoulderYaw = jc.calculate("rCollar","chest", "rShldr")
#rShoulderPitch = jc.calculate("rShldr", "rForeArm", "hip")
#rHipYaw = jc.calculate("rFoot", "End Site", "rThigh")
#rHipRoll = jc.calculate("rThigh", "rShin", "rCollar")
#rHipPitch = jc.calculate("rThigh", "rCollar", "rShin")
rKneePitch = jc.calculate("lShin", "lThigh","lFoot")
#rAnkle = jc.calculate("rFoot", "rShin", "End Site")
print rKneePitch
print rKneePitch.min()
print rKneePitch.max()