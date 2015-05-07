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
        self.rElbowYaw = self.jointCalc.calculateSagital("rForeArm", "rShldr", "rHand") #validado 
        self.framesQty = len(self.rElbowYaw)
        self.frameVectors = {}  
        self.frameVectors["R_Shoulder_Yaw"] = self.jointCalc.calculateTransversal("rCollar","chest", "rShldr") #validado, por ahora no usar o tener cuidado
        self.frameVectors["R_Shoulder_Pitch"] = self.jointCalc.calculateSagital("rShldr", "head", "rForeArm" ) #validado, revisar signo
        self.frameVectors["R_Hip_Yaw"] = self.jointCalc.calculateTransversal("rShin", "rThigh", "rFoot") #validado, por ahora no usar
        self.frameVectors["R_Hip_Roll"] = self.jointCalc.calculateFrontal("rThigh", "abdomen", "rShin")  #validado 
        self.frameVectors["R_Hip_Pitch"] = self.jointCalc.calculateSagital("rThigh", "abdomen", "rShin") #validado
        self.frameVectors["R_Knee"] = self.jointCalc.calculateSagital("rShin", "rThigh","rFoot") #validado
        self.frameVectors["R_Ankle_Pitch"] = self.jointCalc.calculateSagital("rFoot", "rShin", "End Site12") #validado
        self.frameVectors["R_Ankle_Roll"] = self.jointCalc.calculateFrontal("rFoot", "rShin", "End Site12") #self.nullFrameVector(self.framesQty)
        self.frameVectors["R_Elbow_Yaw"] = self.rElbowYaw  #validado
        
        self.frameVectors["L_Shoulder_Yaw"] = self.jointCalc.calculateFrontal("lCollar","chest", "lShldr") #validado 
        self.frameVectors["L_Shoulder_Pitch"] = self.jointCalc.calculateSagital("lShldr", "head", "lForeArm") #validado 
        self.frameVectors["L_Hip_Yaw"] = self.jointCalc.calculateTransversal("lShin", "lThigh", "lFoot") #validado 
        self.frameVectors["L_Hip_Roll"] = self.jointCalc.calculateFrontal("lThigh", "abdomen", "lShin")  #validado 
        self.frameVectors["L_Hip_Pitch"] = self.jointCalc.calculateSagital("lThigh", "abdomen", "lShin") #validado
        self.frameVectors["L_Knee"] = self.jointCalc.calculateSagital("lShin", "lThigh","lFoot") #validado
        self.frameVectors["L_Ankle_Pitch"] = self.jointCalc.calculateSagital("lFoot", "lShin", "End Site13") #validado
        self.frameVectors["L_Ankle_Roll"] =  self.jointCalc.calculateFrontal("lFoot", "lShin", "End Site13") #self.nullFrameVector(self.framesQty)
        self.frameVectors["L_Elbow_Yaw"] = self.jointCalc.calculateSagital("lForeArm", "lShldr", "lHand") #validado
        
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

    def nullFrameVector(self, length):
        frameVector = {}
        for i in range(length):
            frameVector[i]=0
        return frameVector

