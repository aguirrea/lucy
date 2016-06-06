#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Recreating the work done by Reda Al-Bahrani et al @ https://sites.google.com/a/u.northwestern.edu/gawalker
# scene ControllerTest_bonusfs for performance comparison
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
import time
import xml.etree.cElementTree as ET

from Pose                import Pose
from simulator.LoadRobotConfiguration import LoadRobotConfiguration
from simulator.Lucy      import SimulatedLucy

print 'Program started'


#Create Oscillator Parameters
SP_C = 0
SP_A = math.pi/30
SP_Phi = 0

HR_C = 0
HR_A = math.pi/30
HR_Phi = 0

HP_C = math.pi/60
HP_A = -1*math.pi/30
HP_Phi = 0

K_C = 0
K_A = 0
K_Phi = 0

AP_C = 0
AP_A = 0
AP_Phi = 0


T = 0.5
C_TIMESTEP = 0.05
C_TIMEOFFSET = 10

root = ET.Element("root")
lucyPersistence = ET.SubElement(root, "Lucy")

try:
    lucy = SimulatedLucy(True)
    pose = {}
    poses = {}
    poseNumber = 0
    configuration = LoadRobotConfiguration()
    simTimeMark = lucy.getSimTime()
    counter = 0
    isUp = lucy.isLucyUp()
    startTime = time.time()


    '''pose["L_Shoulder_Pitch"] = 0
    pose["R_Shoulder_Pitch"] = 0
    #Hip Roll
    pose["L_Hip_Roll"] = 0
    pose["R_Hip_Roll"] = 0
    #Hip Pitch
    pose["L_Hip_Pitch"] = (26.5/180)*math.pi
    pose["R_Hip_Pitch"] = (26.5/180)*math.pi
    #Knee Pitch
    pose["L_Knee"] = (-50.0/180)*math.pi
    pose["R_Knee"] = (-50.0/180)*math.pi
    #Ankle Pitch
    pose["L_Ankle_Pitch"] = (26.5/180)*math.pi
    pose["R_Ankle_Pitch"] = (26.5/180)*math.pi
    #Ankle Roll
    pose["L_Ankle_Roll"] = 0
    pose["R_Ankle_Roll"] = 0

    newPose = Pose(pose)
    lucy.executeRawPose(newPose)'''

    while isUp and counter <= 400:
        print "executin pose number: ", counter
        frame = ET.SubElement(lucyPersistence, "frame")
        frame.set("number", str(poseNumber))

        simTime = time.time() - startTime
        t = simTime - (C_TIMESTEP * C_TIMEOFFSET)
        print "simTime: ", simTime
        #Calculate Joint Angles
        #Shoulder Pitch
        pose["L_Shoulder_Pitch"] = SP_C+SP_A*math.sin(2*math.pi*t/T+SP_Phi)
        pose["R_Shoulder_Pitch"] = SP_C+SP_A*math.sin(2*math.pi*t/T+SP_Phi+math.pi)
        #Hip Roll
        pose["L_Hip_Roll"] = HR_C+HR_A*math.sin(2*math.pi*t/T+HR_Phi)
        pose["R_Hip_Roll"] = pose["L_Hip_Roll"]
        #Hip Pitch
        pose["L_Hip_Pitch"] = HP_C+HP_A*math.sin(2*math.pi*t/T+HP_Phi)
        print pose["L_Hip_Pitch"]
        pose["R_Hip_Pitch"] = HP_C+HP_A*math.sin(2*math.pi*t/T+HP_Phi+math.pi)
        #Knee Pitch
        pose["L_Knee"] = K_C+K_A*math.sin(2*math.pi*t/T+K_Phi)
        pose["R_Knee"] = RK_Pos=K_C+K_A*math.sin(2*math.pi*t/T+K_Phi+math.pi)
        #Ankle Pitch
        pose["L_Ankle_Pitch"] = AP_C+AP_A*math.sin(2*math.pi*t/T+AP_Phi)
        pose["R_Ankle_Pitch"] = AP_C+AP_A*math.sin(2*math.pi*t/T+AP_Phi+math.pi)
        #Ankle Roll
        pose["L_Ankle_Roll"] = pose["L_Hip_Roll"]
        pose["R_Ankle_Roll"] = pose["L_Ankle_Roll"]

        newPose = Pose(pose)
        lucy.executeRawPose(newPose)
        #time.sleep(0.2)
        poseNumber = poseNumber + 1
        for joint in configuration.getJointsName():
            xmlJoint = ET.SubElement(frame, joint)
            joint_id = configuration.loadJointId(joint)
            pos = newPose.getValue(joint)
            degreeAngle = 150-(pos*float(60))
            xmlJointAngle = xmlJoint.set("angle" , str(degreeAngle))

        counter = counter + 1
        isUp = lucy.isLucyUp()
        print "isUp", isUp
    lucy.stopLucy()
    tree = ET.ElementTree(root)
    tree.write("moon_walk1.xml")
    print 'Program ended'

except Exception, e:
    print str(e)





