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
from LoadSystemConfiguration import LoadSystemConfiguration

import os, threading

X = 0
Y = 1

class SimLucy:

    def __init__(self, visible=False):
        self.conf = LoadSystemConfiguration()
        self.configuration = LoadRobotConfiguration()
        self.visible = visible
        self.sim = Simulator()
        self.clientID = self.sim.connectVREP()
        if self.clientID == -1:
            raise VrepException("error connecting with Vrep", -1)
        genetic_bioloid = os.getcwd()+self.conf.getFile("Lucy vrep model")
        self.sim.loadscn(self.clientID, genetic_bioloid)
        self.sim.startSim(self.clientID,self.visible)
        self.time = 0
        self.startTime = time()
        self.jointHandleCachePopulated = False
        self.stop = False
        self.distance = 0
        configuration = LoadRobotConfiguration()
        self.joints = configuration.getJointsName()
        self.startPosSetted = False
        self.firstCallGetFrame = True
        error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
        if not error:
            self.startPosSetted = True
            self.startPos = [x,y]
        else:
            threading.Timer(int(self.conf.getProperty("threadingTime")), self.setStartPositionAsync).start()

    def setStartPositionAsync(self):
        if not self.startPosSetted:
            error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
            if not error:
                self.startPosSetted = True
                self.startPos = [x,y]
            else:
                threading.Timer(int(self.conf.getProperty("threadingTime")), self.setStartPositionAsync).start()

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
        return self.getSimTime() * self.getSimDistance()
        
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
        dontSupportedJoints = self.conf.getVrepNotImplementedBioloidJoints()
        RobotImplementedJoints = []
        #Above's N joints will be received and set on the V-REP side at the same time'''
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True
        error = self.sim.pauseSim(self.clientID) or error
        robotJoints = self.configuration.getJointsName()
        for joint in robotJoints:
            if joint not in dontSupportedJoints:
                RobotImplementedJoints.append(joint)
        jointsQty = len(RobotImplementedJoints)
        jointExecutedCounter=0
        for joint in RobotImplementedJoints:
            angle = pose.getValue(joint)    
            if jointExecutedCounter < jointsQty - 1:
                error = self.sim.setJointPositionNonBlock(self.clientID, joint, angle) or error
            else:
                error = self.sim.resumePauseSim(self.clientID) or error
                error = self.sim.setJointPosition(self.clientID, joint, angle) or error
            jointExecutedCounter = jointExecutedCounter + 1
        if error:
            raise VrepException("error excecuting a pose", error)

    def getFrame(self): #TODO return a Pose object
        error = False
        pose = {}
        dontSupportedJoints = self.conf.getVrepNotImplementedBioloidJoints()
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True
        error = self.sim.pauseSim(self.clientID) or error
        for joint in self.joints:
            if joint not in dontSupportedJoints: #actual model of vrep bioloid don't support this joints
                errorGetJoint, value = self.sim.getJointPositionNonBlock(self.clientID, joint, self.firstCallGetFrame)
                error = error or errorGetJoint 
                print errorGetJoint
                pose[joint] = value
            else:
                pose[joint] = 0
        self.firstCallGetFrame = False
        error = self.sim.resumePauseSim(self.clientID) or error
        #if error:
        #    raise VrepException("error geting a frame", error)
        return error, pose
        
    def stopLucy(self):
        self.stop = True
        self.time = time() - self.startTime
        errorPosition, x, y = self.sim.getBioloidPlannarPosition(self.clientID) 
        if self.startPosSetted:
            self.distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
        errorFinish = self.sim.finishSimulation(self.clientID)
        #error = errorFinish or errorPosition
        error = errorPosition
        if error:
            raise VrepException("error stoping Lucy", error)        

    def isLucyUp(self):
        error, up = self.sim.isRobotUp(self.clientID)
        if error:
            #raise VrepException("error consulting if lucy is up", error)
            return True
        return up
