#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Backward walk implemented by https://sites.google.com/a/u.northwestern.edu/gawalker
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

from simulator.SimLucy   import SimLucy
from simulator.AXAngle   import AXAngle
from parser.LoadPoses    import LoadPoses
from Pose                import Pose
from simulator.LoadRobotConfiguration import LoadRobotConfiguration
from errors.VrepException import VrepException


import xml.etree.cElementTree as ET

import math
import os
import time


print 'Program started' 


#Create Oscillator Parameters
SP_C = 0
SP_A = math.pi/18
SP_Phi = 0

HR_C = 0
HR_A = 0
HR_Phi = 0

HP_C = -1*math.pi/36
HP_A = math.pi/12
HP_Phi = 0

K_C = -math.pi/12
K_A = math.pi/12
K_Phi = math.pi

AP_C = math.pi/10
AP_A = math.pi/60
AP_Phi = -1*math.pi/12

AR_C = 0
AR_A = 0
AR_Phi = 0

T = 0.5

root = ET.Element("root")
lucyPersistence = ET.SubElement(root, "Lucy")

try:
    lucy = SimLucy(True)
    pose = {}
    poses = {}
    poseNumber = 0
    configuration = LoadRobotConfiguration()
    simTimeMark = lucy.getSimTime()
    while  lucy.isLucyUp():
        frame = ET.SubElement(lucyPersistence, "frame")
        frame.set("number" , str(poseNumber))

        simTime = lucy.getSimTime()
        #Calculate Joint Angles
        #Shoulder Pitch
        pose["L_Shoulder_Pitch"] = SP_C+SP_A*math.sin(2*math.pi*simTime/T+SP_Phi)
        pose["R_Shoulder_Pitch"] = SP_C+SP_A*math.sin(2*math.pi*simTime/T+SP_Phi+math.pi)
        #Hip Roll
        pose["L_Hip_Roll"] = HR_C+HR_A*math.sin(2*math.pi*simTime/T+HR_Phi)
        pose["R_Hip_Roll"] = HR_C+HR_A*math.sin(2*math.pi*simTime/T+HR_Phi+math.pi)
        #Hip Pitch
        pose["L_Hip_Pitch"] = HP_C+HP_A*math.sin(2*math.pi*simTime/T+HP_Phi)
        pose["R_Hip_Pitch"] = HP_C+HP_A*math.sin(2*math.pi*simTime/T+HP_Phi+math.pi)
        #Knee Pitch
        pose["L_Knee"] = K_C+K_A*math.sin(2*math.pi*simTime/T+K_Phi)
        pose["R_Knee"] = RK_Pos=K_C+K_A*math.sin(2*math.pi*simTime/T+K_Phi+math.pi) 
        #Ankle Pitch
        pose["L_Ankle_Pitch"] = AP_C+AP_A*math.sin(2*math.pi*simTime/T+AP_Phi)
        pose["R_Ankle_Pitch"] = AP_C+AP_A*math.sin(2*math.pi*simTime/T+AP_Phi+math.pi)
        #Ankle Roll
        pose["L_Ankle_Roll"] = AR_C+AR_A*math.sin(2*math.pi*simTime/T+AR_Phi)
        pose["R_Ankle_Roll"] = AR_C+AR_A*math.sin(2*math.pi*simTime/T+AR_Phi+math.pi)

        newPose = Pose(pose)
        lucy.executePose(newPose)
        poseNumber = poseNumber + 1
        for joint in configuration.getJointsName():
            xmlJoint = ET.SubElement(frame, joint)
            joint_id = configuration.loadJointId(joint)
            pos = newPose.getValue(joint)
            degreeAngle = 150-(pos*float(60))
            xmlJointAngle = xmlJoint.set("angle" , str(degreeAngle))

    lucy.stopLucy()
    tree = ET.ElementTree(root)
    tree.write("moon_walk1.xml")
    print 'Program ended'

except Exception, e:
    print str(e)





