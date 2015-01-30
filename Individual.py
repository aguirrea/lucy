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
from DTIndividualProperty import DTIndividualProperty
from Pose                 import Pose


class Individual:

    def __init__(self, file):
        self.property = DTIndividualPropertyCMUDaz() #TODO review python herence
        self.fitness = 0
        self.mocapFile = file
        self.lp = LoadPoses(self.mocapFile)
        self.lucy = SimLucy(False)

    def execute(self):
        angleExecute = AXAngle()
        lucyIsDown = False
        while (self.lucy.isLucyUp() and i <= self.lp.getFrameQty()):
            poses = self.lp.getFramePose(i)
            i = i + 1
            for joint in poses.keys():
                if !property.avoidJoint(joint):
                    angleExecute.setDegreeValue(poses[joint] + property.getPoseFix(joint))
                    pose[joint].toVrep()
            self.lucy.executeFrame(pose)
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
            if (newDiff < diff) :
                diff = newDiff
                moreSimilarPose = myPose
        return moreSimilarPose
        

