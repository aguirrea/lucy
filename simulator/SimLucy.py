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

# WARNING, THIS CLASS IS DEPRECATED!

import math
from time import time
from Simulator import Simulator
from LoadRobotConfiguration import LoadRobotConfiguration
from errors.VrepException import VrepException
from Pose import Pose
from LoadSystemConfiguration import LoadSystemConfiguration

import os, threading, time

X = 0
Y = 1

class SimLucy:

    def __init__(self, visible=False):
        self.conf = LoadSystemConfiguration()
        self.configuration = LoadRobotConfiguration()
        self.visible = visible
        genetic_bioloid = os.getcwd()+self.conf.getFile("Lucy vrep model")
        self.sim = Simulator().getInstance(genetic_bioloid)
        self.clientID = self.sim.getClientId() #self.sim.connectVREP()
        if self.clientID == -1:
            raise VrepException("error connecting with Vrep", -1)
        #self.sim.loadscn(self.clientID, genetic_bioloid)
        self.sim.startSim(self.clientID,self.visible)
        self.time = 0
        self.startTime = time.time()
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
            threading.Timer(float(self.conf.getProperty("threadingTime")), self.setStartPositionAsync).start()

    def setStartPositionAsync(self):
        if not self.startPosSetted:
            error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
            if not error:
                self.startPosSetted = True
                self.startPos = [x,y]
            else:
                threading.Timer(float(self.conf.getProperty("threadingTime")), self.setStartPositionAsync).start()

    def getSimTime(self):
        if self.stop == False:
            self.time = time.time() - self.startTime
        return self.time
        
    def getSimDistance(self):
        return self.distance        
    
    def getFitness(self, endFrameExecuted=False):
        #print "time ", self.getSimTime(), "distance ", self.getSimDistance()
        time = self.getSimTime()
        distance = self.getSimDistance()
        #fitness = time + distance * time
        fitness = distance * time
        if endFrameExecuted:
            fitness = fitness * 2
        return fitness
    
    #this function ins deprecated
    def executeFrame(self, pose):
        error = False
        #Above's N joints will be received and set on the V-REP side at the same time'''
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True
        error = self.sim.pauseSim(self.clientID) or error
        for j in xrange(len(pose)-1):
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
        self.updateLucyPosition()
        #if error:
        #    raise VrepException("error excecuting a pose", error)

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
                pose[joint] = value
            else:
                pose[joint] = 0
        self.firstCallGetFrame = False
        error = self.sim.resumePauseSim(self.clientID) or error
        #if error:
        #    raise VrepException("error geting a frame", error)
        return error, pose

    def updateLucyPosition(self):
        if self.stop == False: 
            self.time = time.time() - self.startTime
            errorPosition, x, y = self.sim.getBioloidPlannarPosition(self.clientID) 
            if self.startPosSetted and not errorPosition:
                self.distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
            
    def stopLucy(self):
        self.stop = True
        self.updateLucyPosition()
        self.sim.finishSimulation(self.clientID)
            
    def isLucyUp(self):
        error, up = self.sim.isRobotUp(self.clientID)
        if error:
            #raise VrepException("error consulting if lucy is up", error)
            return True
        return up
