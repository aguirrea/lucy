#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Datatype for the individual property
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

from simulator.SimLucy    import SimLucy
from simulator.AXAngle    import AXAngle
from parser.LoadPoses     import LoadPoses
from DTIndividualProperty import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero
from Pose                 import Pose
from LoadSystemConfiguration import LoadSystemConfiguration
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

import os #only for the tests
import glob #only for the tests
import time

class Individual:

    def __init__(self, idividualProperty, file = None, geneMatrix = None):
        self.property = idividualProperty
        self.fitness = 0
        self.mocapFile = file
        self.robotConfig = LoadRobotConfiguration()
        
        #if the constructor receive a genetic matrix instead a file to create the individual
        if self.mocapFile == None and not(geneMatrix == None):
            self.genomeMatrix = geneMatrix
            self.poseSize = len(geneMatrix)
        #if the constructor receive a file intead a genetic matrix to create the individual
        elif not self.mocapFile == None:
            self.lp = LoadPoses(self.mocapFile)
            self.poseSize = self.lp.getFrameQty()
            self.genomeMatrix = [[self.lp.getPose(i).getValue(j) for j in self.robotConfig.getJointsName()] for i in range(self.poseSize)] 

        self.lucy = SimLucy(True)

        self.genomeMatrixJointNameIDMapping = {}
        i=0
        for jointName in self.robotConfig.getJointsName():
            self.genomeMatrixJointNameIDMapping[jointName]=i
            i=i+1

    def execute(self):
        angleExecute = AXAngle()
        poseExecute={}
        i=0
        while (self.lucy.isLucyUp() and i < self.poseSize):
            for joint in self.robotConfig.getJointsName():
                if not(self.property.avoidJoint(joint)):
                    value = self.genomeMatrix[i][self.genomeMatrixJointNameIDMapping[joint]] + self.property.getPoseFix(joint)
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

prop = DTIndividualPropertyCMUDaz()
propVanilla = DTIndividualPropertyVanilla()
balieroProp = DTIndividualPropertyBaliero()

conf = LoadSystemConfiguration()

CMUxmlDir = os.getcwd()+conf.getDirectory("Transformed CMU mocap Files")
GAwalkDir = os.getcwd()+conf.getDirectory("GAWalk Files")
UIBLHDir = os.getcwd()+conf.getDirectory("UIBLH mocap Files")
BalieroDir = os.getcwd()+conf.getDirectory("Baliero transformed walk Files")
ADHOCDir = os.getcwd()+conf.getDirectory("ADHOC Files")

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

for filename in glob.glob(os.path.join(CMUxmlDir, '*.xml')):
    print 'executing individual: ' + filename
    walk = Individual(prop, filename)
    walk.execute()

#for filename in glob.glob(os.path.join(ADHOCDir, '*.xml')):
#    print 'executing individual: ' + filename
#    walk = Individual(filename, propVanilla)
#    walk.execute()

    
