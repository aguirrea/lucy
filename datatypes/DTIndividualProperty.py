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
        self.avoidJoints = ["L_Elbow_Yaw", "R_Elbow_Yaw", "L_Shoulder_Yaw", "R_Shoulder_Yaw", "R_Hip_Yaw", "L_Hip_Yaw", "L_Ankle_Roll", "R_Ankle_Roll", "L_Ankle_Pitch", "R_Ankle_Pitch", "R_Shoulder_Pitch", "L_Shoulder_Pitch","L_Hip_Roll", "R_Hip_Roll"]    
        self.poseFix["R_Shoulder_Yaw"] = 0
        self.poseFix["R_Shoulder_Pitch"] = -45
        self.poseFix["R_Hip_Yaw"] = -50
        self.poseFix["R_Hip_Roll"] = 0
        self.poseFix["R_Hip_Pitch"] = -50
        self.poseFix["R_Knee"] = 0 
        self.poseFix["R_Ankle_Pitch"] = -126
        self.poseFix["R_Elbow_Yaw"] = 0
        self.poseFix["R_Ankle_Roll"] = -50

        self.poseFix["L_Shoulder_Yaw"] = 0
        self.poseFix["L_Shoulder_Pitch"] = -45
        self.poseFix["L_Hip_Yaw"] = -50
        self.poseFix["L_Hip_Roll"] = -56
        self.poseFix["L_Hip_Pitch"] = -50
        self.poseFix["L_Knee"] = 0
        self.poseFix["L_Ankle_Pitch"] = -127
        self.poseFix["L_Elbow_Yaw"] = 0
        self.poseFix["L_Ankle_Roll"] = 15

class DTIndividualPropertyVanilla(DTIndividualProperty):

    def __init__(self):
        DTIndividualProperty.__init__(self)
        self.avoidJoints = ["R_Elbow_Yaw", "L_Elbow_Yaw", "L_Shoulder_Yaw", "R_Shoulder_Yaw"]
        
        self.poseFix["R_Shoulder_Yaw"] = 0
        self.poseFix["R_Shoulder_Pitch"] = 0
        self.poseFix["R_Hip_Yaw"] = 0
        self.poseFix["R_Hip_Roll"] = 0
        self.poseFix["R_Hip_Pitch"] = 0
        self.poseFix["R_Knee"] = 0 
        self.poseFix["R_Ankle_Pitch"] = 0
        self.poseFix["R_Elbow_Yaw"] = 0
        self.poseFix["R_Ankle_Roll"] = 0

        self.poseFix["L_Shoulder_Yaw"] = 0
        self.poseFix["L_Shoulder_Pitch"] = 0
        self.poseFix["L_Hip_Yaw"] = 0
        self.poseFix["L_Hip_Roll"] = 0
        self.poseFix["L_Hip_Pitch"] = 0
        self.poseFix["L_Knee"] = 0
        self.poseFix["L_Ankle_Pitch"] = 0
        self.poseFix["L_Elbow_Yaw"] = 0
        self.poseFix["L_Ankle_Roll"] = 0

class DTIndividualPropertyBaliero(DTIndividualProperty):

    def __init__(self):
        DTIndividualProperty.__init__(self)
        self.avoidJoints = ["L_Shoulder_Yaw", "R_Shoulder_Yaw", "R_Elbow_Yaw", "L_Elbow_Yaw"] 
        
        self.poseFix["R_Shoulder_Yaw"] = 69
        self.poseFix["R_Shoulder_Pitch"] = 81
        self.poseFix["R_Hip_Yaw"] = 45
        self.poseFix["R_Hip_Roll"] = 4
        self.poseFix["R_Hip_Pitch"] = 54

        self.poseFix["R_Knee"] = 97
        self.poseFix["R_Ankle_Pitch"] = -14
        self.poseFix["R_Elbow_Yaw"] = 15
        self.poseFix["R_Ankle_Roll"] = -39

        self.poseFix["L_Shoulder_Yaw"] = -68
        self.poseFix["L_Shoulder_Pitch"] = -81
        self.poseFix["L_Hip_Yaw"] = -45
        self.poseFix["L_Hip_Roll"] = 1
        self.poseFix["L_Hip_Pitch"] = -43
        self.poseFix["L_Knee"] = -63
        self.poseFix["L_Ankle_Pitch"] = -13
        self.poseFix["L_Elbow_Yaw"] = -14
        self.poseFix["L_Ankle_Roll"] = 41


class DTIndividualPropertyVanillaEvolutive(DTIndividualProperty):

    def __init__(self):
        DTIndividualProperty.__init__(self)
        self.avoidJoints = ["L_Elbow_Yaw", "R_Elbow_Yaw", "L_Shoulder_Yaw", "R_Shoulder_Yaw", "R_Hip_Yaw", "L_Hip_Yaw", "L_Ankle_Roll", "R_Ankle_Roll", "L_Ankle_Pitch", "R_Ankle_Pitch", "R_Shoulder_Pitch", "L_Shoulder_Pitch","L_Hip_Roll", "R_Hip_Roll"]    
        
        self.poseFix["R_Shoulder_Yaw"] = 0
        self.poseFix["R_Shoulder_Pitch"] = 0
        self.poseFix["R_Hip_Yaw"] = 0
        self.poseFix["R_Hip_Roll"] = 0
        self.poseFix["R_Hip_Pitch"] = 0
        self.poseFix["R_Knee"] = 0 
        self.poseFix["R_Ankle_Pitch"] = 0
        self.poseFix["R_Elbow_Yaw"] = 0
        self.poseFix["R_Ankle_Roll"] = 0

        self.poseFix["L_Shoulder_Yaw"] = 0
        self.poseFix["L_Shoulder_Pitch"] = 0
        self.poseFix["L_Hip_Yaw"] = 0
        self.poseFix["L_Hip_Roll"] = 0
        self.poseFix["L_Hip_Pitch"] = 0
        self.poseFix["L_Knee"] = 0
        self.poseFix["L_Ankle_Pitch"] = 0
        self.poseFix["L_Elbow_Yaw"] = 0
        self.poseFix["L_Ankle_Roll"] = 0




