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
from DTIndividualProperty import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla
from Pose                 import Pose
from LoadSystemConfiguration import LoadSystemConfiguration
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

import os #only for the tests
import glob #only for the tests

class Individual:

    def __init__(self, file, idividualProperty):
        self.property = idividualProperty
        self.fitness = 0
        self.mocapFile = file
        self.lp = LoadPoses(self.mocapFile)
        self.lucy = SimLucy(True)
        self.robotConfig = LoadRobotConfiguration()

    def execute(self):
        angleExecute = AXAngle()
        poseExecute={}
        i=0
        while (self.lucy.isLucyUp() and i < self.lp.getFrameQty()):
            pose = self.lp.getPose(i)
            i = i + 1
            for joint in self.robotConfig.getJointsName():
                if not(self.property.avoidJoint(joint)):
                    angleExecute.setDegreeValue(pose.getValue(joint) + self.property.getPoseFix(joint))
                    poseExecute[joint] = angleExecute.toVrep()
            self.lucy.executePose(Pose(poseExecute))
        self.lucy.stopLucy()  
        self.fitness = self.lucy.getFitness()        

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

prop = DTIndividualPropertyCMUDaz()
propVanilla = DTIndividualPropertyVanilla()
conf = LoadSystemConfiguration()
CMUxmlDir = os.getcwd()+conf.getDirectory("Transformed CMU mocap Files")
GAwalkDir = os.getcwd()+conf.getDirectory("GAWalk Files")
UIBLHDir = os.getcwd()+conf.getDirectory("UIBLH mocap Files")

for filename in glob.glob(os.path.join(UIBLHDir, '*.xml')):
    print 'executing individual: ' + filename
    walk = Individual(filename, propVanilla)
    walk.execute()

for filename in glob.glob(os.path.join(GAwalkDir, '*.xml')):
    print 'executing individual: ' + filename
    walk = Individual(filename, propVanilla)
    walk.execute()

for filename in glob.glob(os.path.join(CMUxmlDir, '*.xml')):
    print 'executing individual: ' + filename
    walk = Individual(filename, prop)
    walk.execute()
