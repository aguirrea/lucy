#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Test of loading xml pose files and executing it
#
# Thisc program is free software; you can redistribute it and/or modify
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

from simulator.Simulator import Simulator
from simulator.SimLucy   import SimLucy
from simulator.AXAngle   import AXAngle
from parser.LoadPoses    import LoadPoses

import math
import os
import time

print 'Program started' 
angle = AXAngle()
angleFix = AXAngle()
angleExecute = AXAngle()

mocapFile = os.getcwd()+"/cmu_mocap/xml/test.xml"
lp = LoadPoses(mocapFile)
sim = Simulator()
lucy = SimLucy(True)

poseExecute={}
poseFix={}

poseFix["R_Shoulder_Yaw"] = 0
poseFix["R_Shoulder_Pitch"] = -90
poseFix["R_Hip_Yaw"] = 0
poseFix["R_Hip_Roll"] = 0
poseFix["R_Hip_Pitch"] = 20
poseFix["R_Knee"] = 10
poseFix["R_Ankle_Pitch"] = -5
poseFix["R_Elbow_Yaw"] = 0
poseFix["R_Ankle_Roll"] = 0

poseFix["L_Shoulder_Yaw"] = 0
poseFix["L_Shoulder_Pitch"] = -90
poseFix["L_Hip_Yaw"] = 0
poseFix["L_Hip_Roll"] = 0
poseFix["L_Hip_Pitch"] = 20
poseFix["L_Knee"] = 10
poseFix["L_Ankle_Pitch"] = -5
poseFix["L_Elbow_Yaw"] = 0
poseFix["L_Ankle_Roll"] = 0

avoid_joints = ["R_Hip_Yaw", "R_Shoulder_Yaw", "L_Hip_Yaw", "R_Shoulder_Yaw", "L_Hip_Roll", "R_Hip_Roll", "L_Ankle_Roll", "R_Ankle_Roll", "L_Ankle_Pitch", "R_Ankle_Pitch"] 
frameQty=lp.getFrameQty()
while  True:
    for index in range(frameQty):
        pose=lp.getFramePose(index)
        for joint in pose.keys():
                if joint not in avoid_joints:
                    #print joint, pose[joint]
                    angleExecute.setValue(630 - pose[joint] + poseFix[joint])
                    poseExecute[joint] = angleExecute.toVrep() 
                    #print joint, poseExecute[joint], angleExecute.getValue()  
        #print poseExecute['L_Knee']     
        lucy.executeFrame(poseExecute)
lucy.stopLucy()  
print 'Program ended'




