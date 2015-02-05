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
from LoadRobotConfiguration import LoadRobotConfiguration
from errors.VrepException import VrepException
from Pose import Pose
import os

X = 0
Y = 1

#genetic_bioloid=os.getcwd()+"/simulator/models/ControllerTest_Orig.ttt"
genetic_bioloid=os.getcwd()+"/simulator/models/genetic_bioloid.ttt"

class SimLucy:

    def __init__(self, visible=False):
        self.configuration = LoadRobotConfiguration()
        self.visible = visible
        self.sim = Simulator()
        self.clientID = self.sim.connectVREP()
        if self.clientID == -1:
            raise VrepException("error connecting with Vrep", -1)
        self.sim.loadscn(self.clientID, genetic_bioloid)
        self.sim.startSim(self.clientID,self.visible)
        self.time = 0
        self.startTime = time()
        self.jointHandleCachePopulated = False
        self.stop = False
        configuration = LoadRobotConfiguration()
        self.joints = configuration.getJointsName()
        self.startPosSetted = False
        error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
        if not error:
            self.startPosSetted = True
            self.startPos = [x,y]

    def getSimTime(self):
        if self.stop == False:
            self.time = time() - self.startTime
        return self.time
        
    def getSimDistance(self):
        if self.stop == False:
            error, x, y=self.sim.getBioloidPlannarPosition(self.clientID) 
            if error:
                raise VrepException("error calculating bioloid plannar position", error)
            distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
            self.distance = distance
        return self.distance        
    
    def getFitness(self):
        return self.getSimTime() + self.getSimDistance() * 1000
        
    def executeFrame(self, pose):
        error = False
        #Above's N joints will be received and set on the V-REP side at the same time'''
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True
        error = self.sim.pauseSim(self.clientID) or error
        for j in range(len(pose)-1):
            joint=pose.keys()[j]
            angle=pose[joint]
            error=self.sim.setJointPositionNonBlock(self.clientID, joint, angle) or error
        error = self.sim.resumePauseSim(self.clientID) or error
        joint=pose.keys()[len(pose)-1]
        angle=pose[joint]
        error=self.sim.setJointPosition(self.clientID, joint, angle) or error
        if error:
            raise VrepException("error excecuting a frame", error)

    def executePose(self, pose):
        error = False
        #Above's N joints will be received and set on the V-REP side at the same time'''
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True
        error = self.sim.pauseSim(self.clientID) or error
        joints = self.configuration.getJointsName()
        jointsQty = len(joints)
        jointExecutedCounter=1
        for joint in joints:
            angle = pose.getValue(joint)
            if jointExecutedCounter < jointsQty:
                error = self.sim.setJointPositionNonBlock(self.clientID, joint, angle) or error
            else:
                error = self.sim.resumePauseSim(self.clientID) or error
                error = self.sim.setJointPosition(self.clientID, joint, angle) or error
            jointExecutedCounter = jointExecutedCounter + 1
        if error:
            raise VrepException("error excecuting a pose", error)

    def getFrame(self):
        error = False
        pose = {}
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True
        error = self.sim.pauseSim(self.clientID) or error
        for joint in self.joints:
            errorGetJoint, value = self.sim.getJointPositionNonBlock(self.clientID, joint)
            error = error or errorGetJoint 
            pose[joint] = value
        error = self.sim.resumePauseSim(self.clientID) or error
        if error:
            raise VrepException("error geting a frame", error)
        return pose
        
    def stopLucy(self):
        self.stop = True
        self.time = time() - self.startTime
        errorPosition, x, y = self.sim.getBioloidPlannarPosition(self.clientID) 
        distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
        self.distance = distance
        errorFinish = self.sim.finishSimulation(self.clientID)
        #error = errorFinish or errorPosition
        error = errorPosition
        if error:
            raise VrepException("error stoping Lucy", error)        

    def isLucyUp(self):
        if not self.startPosSetted:
            error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
            if not error:
                self.startPosSetted = True
                self.startPos = [x,y]
        error, up = self.sim.isRobotUp(self.clientID)
        if error:
            #raise VrepException("error consulting if lucy is up", error)
            return True
        return up
