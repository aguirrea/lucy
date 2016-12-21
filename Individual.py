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

import xml.etree.cElementTree as ET

from Pose                                       import Pose
from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
from datatypes.DTGenomeFunctions import DTGenomeFunctions
from datatypes.DTModelRepose                    import DTModelVrepReda
from simulator.LoadRobotConfiguration           import LoadRobotConfiguration
from simulator.Lucy                             import SimulatedLucy, PhysicalLucy


class Individual:

    def __init__(self, idividualProperty, individualGeneticMaterial, modelReposeValue=DTModelVrepReda()):
        self.property = idividualProperty
        self.modelReposeValue = modelReposeValue
        self.fitness = 0
        self.robotConfig = LoadRobotConfiguration()
        self.configuration = LoadSystemConfiguration()
        self.genomeMatrix = individualGeneticMaterial.getGeneticMatrix()
        self.length = individualGeneticMaterial.getLength()
        self.genomeMatrixJointNameIDMapping = {}
        self.sysConf = LoadSystemConfiguration()
        self.concatenationGap = individualGeneticMaterial.getConcatenationGap()
        self.moveArmFirstTime = True
        self.precycleLength = 0
        self.cycleLength = 0

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

        #is important to use only supported joints to avoid errors obtaining the handler of a joint that doesn't exists
        for i in xrange(self.length):
            for joint in self.robotImplementedJoints:
                #print "i: ", i, "j: ", joint
                if self.property.avoidJoint(joint):
                    value = modelReposeValue.getReposeValue(joint)
                else:
                    value = self.genomeMatrix[i][self.genomeMatrixJointNameIDMapping[joint]] + self.property.getPoseFix(joint) #TODO this can be a problem for the physical robot

                self.genomeMatrix[i][self.genomeMatrixJointNameIDMapping[joint]]=value

    def stopLucy(self):
        self.lucy.stopLucy()

    def execute(self):
        #create the corresponding instance of Lucy, depending if it's real or simulated.
        if int(self.sysConf.getProperty("Lucy simulated?"))==1:
            self.lucy = SimulatedLucy(int(self.configuration.getProperty("Lucy render enable")))
        else:
            self.lucy = PhysicalLucy()
   
        poseExecute={}

        i=0
        while (self.lucy.isLucyUp() and i < self.length):
            for joint in self.robotConfig.getJointsName():
                if not(self.property.avoidJoint(joint)):
                    value = self.genomeMatrix[i][self.genomeMatrixJointNameIDMapping[joint]]
                    poseExecute[joint] = value
            i = i + self.lucy.getPosesExecutedByStepQty()
            self.lucy.executePose(Pose(poseExecute))

            '''if self.precycleLength > 0 and i == self.precycleLength - 1:
                self.lucy.moveHelperArm()

            if self.cycleLength > 0 and i == self.cycleLength - 1:
                self.lucy.moveHelperArm()'''

            '''if i > self.precycleLength - 1:
                if self.cycleLength > 0 and (i - self.precycleLength) % self.cycleLength == 0:
                    self.lucy.moveHelperArm()
                    print "---------------------------------------------------moví el brazo"'''
            #self.lucy.moveHelperArm() obtengo peor fitness moviendo el brazo

        self.fitness = self.lucy.getFitness(self.length, self.concatenationGap)
        self.lucy.stopLucy()  #this function also updates time and distance
        return self.fitness

    def getGenomeMatrix(self):
        return self.genomeMatrix

    def setGenomeMatrix(self, geneMatrix):
        self.genomeMatrix = geneMatrix
        dtGenome = DTGenomeFunctions()
        self.length = dtGenome.individualLength(geneMatrix)

    def persist(self,file):
        root = ET.Element("root")
        lucy = ET.SubElement(root, "Lucy")
        for frameIt in xrange(self.length):
            frame = ET.SubElement(lucy, "frame")
            frame.set("number" , str(frameIt))
            for joint in self.robotImplementedJoints: #TODO because the notImplementedJoints doesn't participate in the simulation and fitness evaluation, they are not stored
                xmlJoint = ET.SubElement(frame, joint)
                joint_id = self.robotConfig.loadJointId(joint)
                pos = self.genomeMatrix[frameIt][self.genomeMatrixJointNameIDMapping[joint]]
                xmlJointAngle = xmlJoint.set("angle" , str(pos))
        tree = ET.ElementTree(root)
        tree.write(file)

    def getJointMatrixIDFromName(self, jointName):
        return self.genomeMatrixJointNameIDMapping[jointName]

    def setLength(self, length):
        self.length = length

    def setPrecycleLength(self, precycleLength):
        self.precycleLength = precycleLength

    def setCycleLength(self, cycleLength):
        self.cycleLength = cycleLength

    def getLength(self):
        return self.length

    def getPrecycleLength(self):
        return self.precycleLength

    def getCycleLength(self):
        return self.cycleLength


##Test case:
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

    
