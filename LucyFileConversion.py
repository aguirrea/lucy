#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Transforms from bvh file to lucy mocap file format
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

from JointCalculation import JointCalculation
import xml.etree.cElementTree as ET
from LoadRobotConfiguration import LoadRobotConfiguration

class LucyFileConversion:
    def __init__(self, file):
    	self.file = 'file'
        self.config = LoadRobotConfiguration()
        self.jointCalc = JointCalculation(file)
        self.rElbowYaw = self.jointCalc.calculateSagital("rForeArm", "rShldr", "rHand")
        self.framesQty = len(self.rElbowYaw)
        self.frameVectors = {}
        self.frameVectors["R_Shoulder_Yaw"] = self.jointCalc.calculateFrontal("rCollar","chest", "rShldr")
        self.frameVectors["R_Shoulder_Pitch"] = self.jointCalc.calculateFrontal("rShldr", "rForeArm", "hip")
        self.frameVectors["R_Hip_Yaw"] = self.jointCalc.calculateFrontal("End Site","rFoot", "rShin")
        self.frameVectors["R_Hip_Roll"] = self.jointCalc.calculateFrontal("rThigh","hip" , "rShin")  #validado ?
        self.frameVectors["R_Hip_Pitch"] = self.jointCalc.calculateSagital("rThigh", "abdomen", "rShin") #validado
        self.frameVectors["R_Knee"] = self.jointCalc.calculateSagital("rShin", "rThigh","rFoot") #validado
        self.frameVectors["R_Ankle_Pitch"] = self.jointCalc.calculateSagital("rFoot", "rShin", "End Site") #validado
        self.frameVectors["R_Elbow_Yaw"] = self.frameVectors["R_Ankle_Pitch"]
        self.frameVectors["R_Ankle_Roll"] = self.frameVectors["R_Ankle_Pitch"]

        self.frameVectors["L_Shoulder_Yaw"] = self.jointCalc.calculateFrontal("lCollar","chest", "lShldr")
        self.frameVectors["L_Shoulder_Pitch"] = self.jointCalc.calculateFrontal("lShldr", "lForeArm", "hip")
        self.frameVectors["L_Hip_Yaw"] = self.jointCalc.calculateFrontal("lFoot", "End Site", "lThigh")
        self.frameVectors["L_Hip_Roll"] = self.jointCalc.calculateFrontal("lThigh","hip" , "lShin")  #validado ?
        self.frameVectors["L_Hip_Pitch"] = self.jointCalc.calculateSagital("lThigh", "abdomen", "lShin") #validado
        self.frameVectors["L_Knee"] = self.jointCalc.calculateSagital("lShin", "lThigh","lFoot") #validado
        self.frameVectors["L_Ankle_Pitch"] = self.jointCalc.calculateSagital("lFoot", "lShin", "End Site") #validado
        self.frameVectors["L_Elbow_Yaw"] = self.frameVectors["L_Ankle_Pitch"]
        self.frameVectors["L_Ankle_Roll"] = self.frameVectors["L_Ankle_Pitch"]
        
    def generateFile(self,file):
        root = ET.Element("root")
        lucy = ET.SubElement(root, "Lucy")
        for frameIt in range(self.framesQty):
            frame = ET.SubElement(lucy, "frame")
            frame.set("number" , str(frameIt))
            for joint in self.config.getJointsName():
                xmlJoint = ET.SubElement(frame, joint)
                joint_id = self.config.loadJointId(joint)
                pos = self.frameVectors[joint][frameIt]
                xmlJointAngle = xmlJoint.set("angle" , str(pos))
        tree = ET.ElementTree(root)
        tree.write(file)

fc = LucyFileConversion("02_02.bvh")
fc.generateFile("test.xml")