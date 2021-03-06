#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Services for accesing to the persisted poses files
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

from xml.dom import minidom
from Pose import Pose
from configuration.LoadSystemConfiguration import LoadSystemConfiguration
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

class LoadPoses:
    
    def __init__(self, poseFile):
        xmldoc = minidom.parse(poseFile)
        self.framelist = xmldoc.getElementsByTagName("frame") 
        self.jointAngleMapping = {}
        self.frameAngleMapping = {}   
    
    #warning this method is deprecated by getPose    
    def getFramePose(self, frameNumber):
        config = LoadRobotConfiguration()
        frame = self.framelist[frameNumber]
        for jointName in config.getJointsName():
            joint = frame.getElementsByTagName(jointName)[0]
            angle = joint.getAttribute("angle")
            self.jointAngleMapping[jointName] = float(angle)
        return(self.jointAngleMapping)
    
    #to deprecate getFramePose
    def getPose(self, frameNumber):
        sysconf = LoadSystemConfiguration()
        robotConf = LoadRobotConfiguration()
        dontSupportedJoints = sysconf.getVrepNotImplementedBioloidJoints()
        robotImplementedJoints = []
        robotJoints = robotConf.getJointsName()

        for joint in robotJoints:
            if joint not in dontSupportedJoints:
                robotImplementedJoints.append(joint)

        frame = self.framelist[frameNumber]
        for jointName in robotImplementedJoints:
            joint = frame.getElementsByTagName(jointName)[0]
            angle = joint.getAttribute("angle")
            self.jointAngleMapping[jointName] = float(angle)
        return(Pose(self.jointAngleMapping))    

    def getJointAngles(self, jointName):
        config = LoadRobotConfiguration()
        for frame in self.framelist:
            frameNumber = frame.getAttribute("number")
            joint = frame.getElementsByTagName(jointName)[0]
            angle = joint.getAttribute("angle")
            self.frameAngleMapping[int(frameNumber)] = float(angle) 
        return(self.frameAngleMapping)

    def getFrameQty(self):
        return self.framelist.length
    
#lp = LoadPoses()
#print lp.getFramePose(1)
#print lp.getJointAngles("R_Ankle_Pitch")
#print lp.getJointAngles("R_Ankle_Roll")
#frameQty=lp.getFrameQty()
#print "there is " + str(frameQty) + " frames."
