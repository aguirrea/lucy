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
from time                           import time
from Simulator                      import Simulator
from LoadRobotConfiguration         import LoadRobotConfiguration
from errors.VrepException           import VrepException
from Pose                           import Pose
from LoadSystemConfiguration        import LoadSystemConfiguration
from datatypes.DTIndividualProperty import DTIndividualProperty, DTIndividualPropertyPhysicalBioloid
from Communication                  import CommSerial
import Actuator
from AXAngle                        import AXAngle

import os, threading, time

X = 0
Y = 1
MAX_FITNESS_BONUS = 5

#abstract class representing lucy abstraction base class
class Lucy(object):
    def __init__(self):
        self.sysConf = LoadSystemConfiguration()
        self.robotConfiguration = LoadRobotConfiguration()
        self.joints = self.robotConfiguration.getJointsName()
        self.time = 0
        self.startTime = time.time()
        self.distance = 0
        self.stop = False
        self.poseExecuted = 0

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
        Lucy.__init__(self)
        self.comm_tty = CommSerial() 
        self.comm_tty.connect()
        self.actuator = Actuator.Actuator(self.comm_tty)
        self.defaultSpeed = 600 #TODO change this, use configuration files
        self.initialPoses = {}

        #checking communication with motors
        for joint in self.joints:
            jointID = self.robotConfiguration.loadJointId(joint)
            print jointID
            self.initialPoses[joint] = self.actuator.get_position(jointID).toDegrees()
            print jointID, joint, self.initialPoses[joint]
            time.sleep(0.1)
            self.actuator.led_state_change(jointID, 1)
            time.sleep(0.1)
        
        time.sleep(1)
        
        for joint in self.joints:
            self.actuator.led_state_change(self.robotConfiguration.loadJointId(joint), 0)
        
    def executePose(self, pose):
        #set positions and wait that the actuator reaching that position
        dontSupportedJoints = self.sysConf.getVrepNotImplementedBioloidJoints()
        RobotImplementedJoints = []
        for joint in self.joints:
            if joint not in dontSupportedJoints:
                RobotImplementedJoints.append(joint)
        for joint in RobotImplementedJoints:
            angle = pose.getValue(joint)
            angleAX = AXAngle()   
            angleAX.setDegreeValue(angle)
            #TODO implement method for setting position of all actuators at the same time
            self.actuator.move_actuator(self.robotConfiguration.loadJointId(joint), int(angleAX.getValue()), self.defaultSpeed)
            self.poseExecuted = self.poseExecuted + 1
        time.sleep(3)

    def stopLucy(self):
        for joint in self.joints:
            self.actuator.move_actuator(self.robotConfiguration.loadJointId(joint), self.initialPoses[joint], self.defaultSpeed)

    def getFrame(self):
        error = False
        pose = {}
        for joint in self.joints:
            value = self.actuator.get_position(self.robotConfiguration.loadJointId(joint))
            pose[joint] = value
        return error, pose            

    def getFitness(self, endFrameExecuted=False):
        return 0

    def isLucyUp(self):
        return True


#Lucy instanciated in a third party Bioloid Premium robot model
class SimulatedLucy(Lucy):

    def __init__(self, visible=False):
        Lucy.__init__(self)
        self.visible = visible
        genetic_bioloid = os.getcwd() + self.sysConf.getFile("Lucy vrep model")
        self.sim = Simulator().getInstance(genetic_bioloid)
        self.clientID = self.sim.getClientId() 
        if self.clientID == -1:
            raise VrepException("error connecting with Vrep", -1)
        self.sim.startSim(self.clientID,self.visible)
        self.jointHandleCachePopulated = False
        configuration = LoadRobotConfiguration()
        self.joints = configuration.getJointsName()
        self.startPosSetted = False
        self.firstCallGetFrame = True
        error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
        if not error:
            self.startPosSetted = True
            self.startPos = [x,y]
        else:
            threading.Timer(float(self.sysConf.getProperty("threadingTime")), self.setStartPositionAsync).start()

    def setStartPositionAsync(self):
        if not self.startPosSetted:
            error, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
            if not error:
                self.startPosSetted = True
                self.startPos = [x,y]
            else:
                threading.Timer(float(self.sysConf.getProperty("threadingTime")), self.setStartPositionAsync).start()

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
        print "distance traveled: ", distance
        #fitness = time + distance * time
        #fitness = distance**2 * time
        #fitness = distance * self.poseExecuted
        #print "distance: ", distance, "poseExecuted: ", self.poseExecuted
        framesQty = int(self.sysConf.getProperty("Individual frames quantity")) 
        fitness = distance * self.poseExecuted/framesQty 
        #if endFrameExecuted:
            #fitness = fitness * 2
        return fitness

    def executePose(self, pose):
        error = False
        dontSupportedJoints = self.sysConf.getVrepNotImplementedBioloidJoints()
        RobotImplementedJoints = []
        #Above's N joints will be received and set on the V-REP side at the same time

        #TODO this must be checked in the simulator class
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True

        error = self.sim.pauseSim(self.clientID) or error
        for joint in self.joints:
            if joint not in dontSupportedJoints:
                RobotImplementedJoints.append(joint)

        jointsQty = len(RobotImplementedJoints)
        jointExecutedCounter=0
        for joint in RobotImplementedJoints:
            angle = pose.getValue(joint) 
            angleAX = AXAngle()   
            angleAX.setDegreeValue(angle)   

            if jointExecutedCounter < jointsQty - 1:
                error = self.sim.setJointPositionNonBlock(self.clientID, joint, angleAX.toVrep()) or error
            else:
                error = self.sim.resumePauseSim(self.clientID) or error
                error = self.sim.setJointPosition(self.clientID, joint, angleAX.toVrep()) or error

            jointExecutedCounter = jointExecutedCounter + 1

        self.updateLucyPosition()
        self.poseExecuted = self.poseExecuted + 1
        #if error:
        #    raise VrepException("error excecuting a pose", error)

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
            #print "x_position: ", x, "y_position: ", y 
            if self.startPosSetted and not errorPosition:
                #self.distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
                distToGoal = math.sqrt((x-1)**2 + (y-0)**2)
                distToGoal = 1 - distToGoal
                if distToGoal < 0 :
                    self.distance = 0
                else: 
                    self.distance = distToGoal
            
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
