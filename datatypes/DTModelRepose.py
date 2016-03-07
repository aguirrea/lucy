#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Datatype for representing the repose value of each joint in an instance of a simulator
# see mocap/tests/repose_cmu_model.xml
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

from simulator.LoadRobotConfiguration import LoadRobotConfiguration

class DTModelRepose(object):
    def __init__(self):
        self.repose={}
        self.robotConfiguration = LoadRobotConfiguration()
        self.joints = self.robotConfiguration.getJointsName()

    def getReposeValue(self, joint):
        if joint in self.repose.keys():
            res = self.repose[joint]
        else:
            res = 0
        return res


class DTModelVrepReda(DTModelRepose):
    
    def __init__(self):
        DTModelRepose.__init__(self)
        self.repose["R_Shoulder_Yaw"] = 150
        self.repose["R_Shoulder_Pitch"] = 149.4
        self.repose["R_Hip_Yaw"] = 150
        self.repose["R_Hip_Roll"] = 150
        self.repose["R_Hip_Pitch"] = 154.4
        self.repose["R_Knee"] = 166
        self.repose["R_Ankle_Pitch"] = 131.8
        self.repose["R_Elbow_Yaw"] = 150
        self.repose["R_Ankle_Roll"] = 150

        self.repose["L_Shoulder_Yaw"] = 150
        self.repose["L_Shoulder_Pitch"] = 150.5
        self.repose["L_Hip_Yaw"] = 150
        self.repose["L_Hip_Roll"] = 150 
        self.repose["L_Hip_Pitch"] = 156.1
        self.repose["L_Knee"] = 165
        self.repose["L_Ankle_Pitch"] = 130.5
        self.repose["L_Elbow_Yaw"] = 150
        self.repose["L_Ankle_Roll"] = 150
    

