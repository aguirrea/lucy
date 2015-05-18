#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Lucy representation.
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
from dataypes.DTIndividualProperty import DTIndividualPropertyPhysicalBioloid

import os, threading, time

from Communication import CommSerial
import Actuator

X = 0
Y = 1
MAX_FITNESS_BONUS = 5

#abstract class representing lucy abstraction base class
class Lucy(object):
    def __init__(self):
        self.sysConf = LoadSystemConfiguration
        self.robotConfiguration = LoadRobotConfiguration()
        self.joints = self.robotConfiguration.getJointsName()
        self.time = 0
        self.startTime = time()
        self.distance = 0
        self.stoped = False

    def getFitness(self, endFrameExecuted=False):
        pass

    def executePose(self, pose):
        pass

    def getFrame(self):
        pass

    def stopLucy(self):
        pass

    def isLucyUp(self):
        pass

#Lucy instanciated in a Bioloid Premium robot
class PhysicalLucy(Lucy):

    def __init__(self):
        self.comm_tty = CommSerial() 
        self.comm_tty.connect()
        self.actuator = Actuator.Actuator(comm_tty)
        self.defaultSpeed = 600 #TODO change this, use configuration files
        self.bioloidProperty = DTIndividualPropertyPhysicalBioloid()

        #checking communication with motors
        for joint in self.joints:
            self.actuator.led_state_change(loadJointId(joint), 1)
        time.sleep(1)
        for joint in self.joints:
            self.actuator.led_state_change(loadJointId(joint), 0)
        
    def executePose(self, pose):
        #set positions and wait that the actuator reaching that position
        for j in xrange(len(pose)-1):
            joint=pose.keys()[j]
            angle=pose[joint]
            #TODO implement method for setting position of all actuators at the same time
            self.actuator.move_actuator(loadJointId(joint), angle, self.defaultSpeed)

    def stopLucy(self):
        for joint in self.joints:
            self.actuator.move_actuator(loadJointId(joint), self.bioloidProperty().getPoseFix(joint), self.defaultSpeed)

    def getFrame(self):
        error = False
        pose = {}
        for joint in self.joints:
            value = self.actuator.get_position(loadJointId(joint))
            pose[joint] = value
        return error, pose            

    def getFitness(self, endFrameExecuted=False):
        return 0

    def isLucyUp(self):
        return True


#Lucy instanciated in a third party Bioloid Premium robot model
class SimulatedLucy(Lucy):

    def __init__(self, visible=False):
        self.visible = visible
        self.sim = Simulator().getInstance()
        self.clientID = self.sim.connectVREP()
        if self.clientID == -1:
            raise VrepException("error connecting with Vrep", -1)
        genetic_bioloid = os.getcwd()+self.sysConf.getFile("Lucy vrep model")
        self.sim.loadscn(self.clientID, genetic_bioloid)
        self.sim.startSim(self.clientID,self.visible)
        self.jointHandleCachePopulated = False
        self.startPosSetted = False
        self.firstCallGetFrame = True
        error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
        if not error:
            self.startPosSetted = True
            self.startPos = [x,y]
        else:
            threading.Timer(int(self.sysConf.getProperty("threadingTime")), self.setStartPositionAsync).start()

    def setStartPositionAsync(self):
        if not self.startPosSetted:
            error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
            if not error:
                self.startPosSetted = True
                self.startPos = [x,y]
            else:
                threading.Timer(int(self.sysConf.getProperty("threadingTime")), self.setStartPositionAsync).start()

    def getSimTime(self):
        if self.stoped == False:
            self.time = time() - self.startTime
        return self.time
        
    def getSimDistance(self):
        if self.stoped == False:
            error, x, y=self.sim.getBioloidPlannarPosition(self.clientID) 
            if error:
                raise VrepException("error calculating bioloid plannar position", error)
            distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
            self.distance = distance
        return self.distance        
    
    def getFitness(self, endFrameExecuted=False):
        #print "time ", self.getSimTime(), "distance ", self.getSimDistance()
        time = self.getSimTime()
        distance = self.getSimDistance()
        fitness = time + distance * time
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
        dontSupportedJoints = self.sysConf.getVrepNotImplementedBioloidJoints()
        RobotImplementedJoints = []
        #Above's N joints will be received and set on the V-REP side at the same time'''
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True
        error = self.sim.pauseSim(self.clientID) or error
        robotJoints = self.robotConfiguration.getJointsName()
        for joint in robotJoints:
            if joint not in dontSupportedJoints:
                RobotImplementedJoints.append(joint)
        jointsQty = len(RobotImplementedJoints)
        jointExecutedCounter=0
        #is important to use only supported joints to avoid errors obtaining the handler of a joint that doesn't exists
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
        dontSupportedJoints = self.sysConf.getVrepNotImplementedBioloidJoints()
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
        self.stoped = True
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
