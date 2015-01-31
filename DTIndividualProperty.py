#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Datatype for the individual property
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

class DTIndividualProperty(object):
    def __init__(self):
        self.poseFix={}
        self.avoidJoints=[]

    def avoidJoint(self, joint):
        return joint in self.avoidJoints

    def getPoseFix(self, joint):
        if joint in self.poseFix.keys():
            res = self.poseFix[joint]
        else:
            res = 0
        return res

class DTIndividualPropertyCMUDaz(DTIndividualProperty):

    def __init__(self):
        DTIndividualProperty.__init__(self)
        self.avoidJoints = ["R_Hip_Yaw", "L_Shoulder_Yaw", "L_Hip_Yaw", "R_Shoulder_Yaw", "L_Hip_Roll", "R_Hip_Roll", "L_Ankle_Roll", "R_Ankle_Roll"] 
        
        self.poseFix["R_Shoulder_Yaw"] = 0
        self.poseFix["R_Shoulder_Pitch"] = -45
        self.poseFix["R_Hip_Yaw"] = 0
        self.poseFix["R_Hip_Roll"] = 0
        self.poseFix["R_Hip_Pitch"] = -70
        self.poseFix["R_Knee"] = 0 
        self.poseFix["R_Ankle_Pitch"] = 9
        self.poseFix["R_Elbow_Yaw"] = 0
        self.poseFix["R_Ankle_Roll"] = 0

        self.poseFix["L_Shoulder_Yaw"] = 0
        self.poseFix["L_Shoulder_Pitch"] = -70
        self.poseFix["L_Hip_Yaw"] = 0
        self.poseFix["L_Hip_Roll"] = 0
        self.poseFix["L_Hip_Pitch"] = -50
        self.poseFix["L_Knee"] = 0
        self.poseFix["L_Ankle_Pitch"] = 9
        self.poseFix["L_Elbow_Yaw"] = 0
        self.poseFix["L_Ankle_Roll"] = 0

