#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Lucy interaction with the simulator
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

standadRemoteApiPort=19997
localhost='127.0.0.1'
genetic_bioloid=os.getcwd()+"/models/genetic_bioloid.ttt"

import math
from time import time
from Simulator import Simulator
X = 0
Y = 1

class SimLucy:

    def __init__(self, visible=False):
        self.sim = Simulator()
        self.clientID = self.connectVREP()
        self.time = time()
        x,y = sim.getBioloidPlannarPosition(self.clientID)
        self.startPos = [x,y]
        
    def getSimTime(self):
        return time() - self.time
        
    def getSimDistance(self):
        x, y=sim.getBioloidPlannarPosition(self.clientID) 
        distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
        return distance
    
    def getFitness(self):
        retrun self.getSimTime + self.getSimDistance() * 1000
        
    def executeFrame(self):
        #Above's 3 joints will be received and set on the V-REP side at the same time'''
        sim.pauseSim(self.clientID)
        #vrep.simxSetJointPosition(clientID,joint1Handle,joint1Value,vrep.simx_opmode_oneshot)
        #vrep.simxSetJointPosition(clientID,joint2Handle,joint2Value,vrep.simx_opmode_oneshot)
        #vrep.simxSetJointPosition(clientID,joint3Handle,joint3Value,vrep.simx_opmode_oneshot)
        sim.resumePauseSim(self.clientID)
        #TODO
        pass
    
    
