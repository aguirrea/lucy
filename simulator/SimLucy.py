#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Lucy interaction with the simulator.
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
from time import time
from Simulator import Simulator
import os
X = 0
Y = 1
standadRemoteApiPort=19997
localhost='127.0.0.1'
genetic_bioloid=os.getcwd()+"/simulator/models/genetic_bioloid.ttt"

class SimLucy:

    def __init__(self, visible=False):
        self.visible = visible
        self.sim = Simulator()
        self.clientID = self.sim.connectVREP()
        self.sim.loadscn(self.clientID, genetic_bioloid)
        self.sim.startSim(self.clientID,self.visible)
        self.time = 0
        self.startTime = time()
        x,y = self.sim.getBioloidPlannarPosition(self.clientID)
        self.startPos = [x,y]
        self.jointHandleCachePopulated = False
        self.stop = False
        
    def getSimTime(self):
        if self.stop == False:
            time() - self.startTime
        return self.time
        
    def getSimDistance(self):
        if self.stop == False:
            x, y=self.sim.getBioloidPlannarPosition(self.clientID) 
            distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
            self.distance = distance
        return self.distance        
    
    def getFitness(self):
        return self.getSimTime() + self.getSimDistance() * 1000
        
    def executeFrame(self, pose):
        #Above's N joints will be received and set on the V-REP side at the same time'''
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True
        self.sim.pauseSim(self.clientID)
        for j in range(len(pose)-1):
            joint=pose.keys()[j]
            angle=pose[joint]
            self.sim.setJointPositionNonBlock(self.clientID, joint, angle)
        self.sim.resumePauseSim(self.clientID)
        joint=pose.keys()[len(pose)-1]
        angle=pose[joint]
        self.sim.setJointPosition(self.clientID, joint, angle)
        #print "pase"
        
        

    def stopLucy(self):
        self.sim.finishSimulation(self.clientID)
        self.stop = True
        self.time = time() - self.startTime
        x, y=self.sim.getBioloidPlannarPosition(self.clientID) 
        distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
        self.distance = distance        
