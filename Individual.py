#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Genetic Algorithm individual representation
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

from simulator.SimLucy                import SimLucy
from simulator.AXAngle                import AXAngle
from parser.LoadPoses                 import LoadPoses
from DTIndividualProperty             import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero
from DTIndividualGeneticMaterial      import DTIndividualGeneticMaterial, DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix
from Pose                             import Pose
from LoadSystemConfiguration          import LoadSystemConfiguration
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

import os #only for the tests
import glob #only for the tests
import time
import xml.etree.cElementTree as ET

class Individual:

    def __init__(self, idividualProperty, individualGeneticMaterial):
        self.property = idividualProperty
        self.fitness = 0

        self.robotConfig = LoadRobotConfiguration()
        self.configuration = LoadSystemConfiguration()
        
        self.genomeMatrix = individualGeneticMaterial.getGeneticMatrix()
        self.poseSize = len(self.genomeMatrix)

        self.lucy = SimLucy(True)

        self.genomeMatrixJointNameIDMapping = {}
        i=0
        for jointName in self.robotConfig.getJointsName():
            self.genomeMatrixJointNameIDMapping[jointName]=i
            i=i+1

        dontSupportedJoints = self.configuration.getVrepNotImplementedBioloidJoints()
        self.robotImplementedJoints = []
        robotJoints = self.robotConfig.getJointsName()
        for joint in robotJoints:
            if joint not in dontSupportedJoints:
                self.robotImplementedJoints.append(joint)

        for i in range(self.poseSize):
            for joint in self.robotImplementedJoints:
                value = self.genomeMatrix[i][self.genomeMatrixJointNameIDMapping[joint]] + self.property.getPoseFix(joint)
                self.genomeMatrix[i][self.genomeMatrixJointNameIDMapping[joint]]=value

    def execute(self):
        angleExecute = AXAngle()
        poseExecute={}
        i=0
        while (self.lucy.isLucyUp() and i < self.poseSize):
            for joint in self.robotConfig.getJointsName():
                if not(self.property.avoidJoint(joint)):
                    value = self.genomeMatrix[i][self.genomeMatrixJointNameIDMapping[joint]]
                    angleExecute.setDegreeValue(value)
                    poseExecute[joint] = angleExecute.toVrep() 
            i = i + 1  
            self.lucy.executePose(Pose(poseExecute))
        self.lucy.stopLucy()  
        self.fitness = self.lucy.getFitness()
        print "fitness: ", self.fitness
        return self.fitness       
         
    def getPoseQty(self):
        return self.lp.getFrameQty()

    def getPose(self, poseNumber):
        return self.lp.getFramePose(poseNumber) 

    def getMostSimilarPose(self, pose):
        diff = MAX_INT 
        moreSimilarPose = self.getPose(1)
        for i in range(self.getPoseQty()):
            myPose = getPose(i)
            newDiff = pose.diff(myPose)
            if (newDiff < diff):
                diff = newDiff
                moreSimilarPose = myPose
        return moreSimilarPose

    def getGenomeMatrix(self):
        return self.genomeMatrix

    def setGenomeMatrix(self, geneMatrix):
        self.genomeMatrix = geneMatrix
        self.poseSize = len(geneMatrix)

    def persist(self,file):
        root = ET.Element("root")
        lucy = ET.SubElement(root, "Lucy")
        for frameIt in range(self.poseSize):
            frame = ET.SubElement(lucy, "frame")
            frame.set("number" , str(frameIt))
            for joint in self.robotImplementedJoints:
                xmlJoint = ET.SubElement(frame, joint)
                joint_id = self.robotConfig.loadJointId(joint)
                pos = self.genomeMatrix[frameIt][self.genomeMatrixJointNameIDMapping[joint]]
                xmlJointAngle = xmlJoint.set("angle" , str(pos))
        tree = ET.ElementTree(root)
        tree.write(file)

#prop = DTIndividualPropertyCMUDaz()
#propVanilla = DTIndividualPropertyVanilla()
#balieroProp = DTIndividualPropertyBaliero()

#conf = LoadSystemConfiguration()

#CMUxmlDir = os.getcwd()+conf.getDirectory("Transformed CMU mocap Files")
#GAwalkDir = os.getcwd()+conf.getDirectory("GAWalk Files")
#UIBLHDir = os.getcwd()+conf.getDirectory("UIBLH mocap Files")
#BalieroDir = os.getcwd()+conf.getDirectory("Baliero transformed walk Files")
#ADHOCDir = os.getcwd()+conf.getDirectory("ADHOC Files")

#for filename in glob.glob(os.path.join(GAwalkDir, '*.xml')):
#    print 'executing individual: ' + filename
#    walk = Individual(filename, propVanilla)
#    walk.execute()

#for filename in glob.glob(os.path.join(BalieroDir, '*.xml')):
#    print 'executing individual: ' + filename
#    walk = Individual(filename, balieroProp)
#    walk.execute()

#for filename in glob.glob(os.path.join(UIBLHDir, '*.xml')):
#    print 'executing individual: ' + filename
#    walk = Individual(filename, propVanilla)
#    walk.execute()

#for filename in glob.glob(os.path.join(GAwalkDir, '*.xml')):
#    print 'executing individual: ' + filename
#    walk = Individual(filename, propVanilla)
#    walk.execute()

#for filename in glob.glob(os.path.join(CMUxmlDir, '*.xml')):
#    print 'executing individual: ' + filename
#    walk = Individual(prop, DTIndividualGeneticTimeSerieFile(filename))
#    walk.execute()
#    walk.persist("borrame.xml")

#for filename in glob.glob(os.path.join(ADHOCDir, '*.xml')):
#    print 'executing individual: ' + filename
#    walk = Individual(filename, propVanilla)
#    walk.execute()

    
