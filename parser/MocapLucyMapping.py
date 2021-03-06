#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Transforms from cmu bvh file to lucy mocap file format
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

from parser.JointCalculation import JointCalculation
import xml.etree.cElementTree as ET

class MocapLucyMapping:
    def __init__(self, file, robotConfiguration):
        self.file = 'file'
        self.config = robotConfiguration
        self.jointCalc = JointCalculation(file)
        #TODO take into account that the elbowyaw can change from sagital plane to transversal
        self.rElbowYaw = self.jointCalc.calculateRightSagital("rForeArm", "rShldr", "rHand")
        self.framesQty = len(self.rElbowYaw)
        self.frameVectors = {}

        self.frameVectors["R_Shoulder_Yaw"] = self.jointCalc.calculateTransversal("rCollar","chest", "rShldr")
        self.frameVectors["R_Shoulder_Pitch"] = 360 - self.jointCalc.calculateRightSagital("rShldr", "rForeArm", "abdomen" ) #swaped
        self.frameVectors["R_Hip_Yaw"] = self.jointCalc.calculateTransversal("rShin", "rThigh", "rFoot")
        self.frameVectors["R_Hip_Roll"] = self.jointCalc.calculateFrontal("rThigh", "abdomen", "rShin")
        self.frameVectors["R_Hip_Pitch"] = self.jointCalc.calculateLeftSagital("rThigh", "abdomen", "rShin")
        self.frameVectors["R_Knee"] = self.jointCalc.calculateLeftSagital("rShin", "rThigh","rFoot")
        self.frameVectors["R_Ankle_Pitch"] = self.jointCalc.calculateLeftSagital("rFoot", "rShin", "End Site12")
        self.frameVectors["R_Ankle_Roll"] = self.jointCalc.calculateFrontal("rFoot", "rShin", "End Site12")
        self.frameVectors["R_Elbow_Yaw"] = self.rElbowYaw
        
        self.frameVectors["L_Shoulder_Yaw"] = self.jointCalc.calculateFrontal("lCollar","chest", "lShldr")
        self.frameVectors["L_Shoulder_Pitch"] = 360 - self.jointCalc.calculateLeftSagital("lShldr", "lForeArm", "abdomen") #swaped
        self.frameVectors["L_Hip_Yaw"] = self.jointCalc.calculateTransversal("lShin", "lThigh", "lFoot")
        self.frameVectors["L_Hip_Roll"] = self.jointCalc.calculateFrontal("lThigh", "abdomen", "lShin")
        self.frameVectors["L_Hip_Pitch"] = self.jointCalc.calculateLeftSagital("lThigh", "abdomen", "lShin")
        self.frameVectors["L_Knee"] = self.jointCalc.calculateLeftSagital("lShin", "lThigh","lFoot")
        self.frameVectors["L_Ankle_Pitch"] = self.jointCalc.calculateLeftSagital("lFoot", "lShin", "End Site13")
        self.frameVectors["L_Ankle_Roll"] =  self.jointCalc.calculateFrontal("lFoot", "lShin", "End Site13")
        self.frameVectors["L_Elbow_Yaw"] = self.jointCalc.calculateLeftSagital("lForeArm", "lShldr", "lHand")
        
    def generateFile(self,file):
        root = ET.Element("root")
        lucy = ET.SubElement(root, "Lucy")
        for frameIt in xrange(self.framesQty):
            frame = ET.SubElement(lucy, "frame")
            frame.set("number" , str(frameIt))
            for joint in self.config.getJointsName():
                xmlJoint = ET.SubElement(frame, joint)
                joint_id = self.config.loadJointId(joint)
                pos = self.frameVectors[joint][frameIt]
                xmlJointAngle = xmlJoint.set("angle" , str(pos))
        tree = ET.ElementTree(root)
        tree.write(file)

    def nullFrameVector(self, length):
        frameVector = {}
        for i in xrange(length):
            frameVector[i]=0
        return frameVector

