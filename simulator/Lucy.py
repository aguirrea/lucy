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

import os
import threading
import time
from collections import Counter
from numpy import angle
from numpy import conjugate

from errors.VrepException           import VrepException

import Actuator
from AXAngle                        import AXAngle
from Communication                  import CommSerial
from LoadRobotConfiguration         import LoadRobotConfiguration
from LoadSystemConfiguration        import LoadSystemConfiguration
from Simulator                      import Simulator

X = 0
Y = 1

BALANCE_HEIGHT = 0.214 #Distance from the floor when lucy is straight up

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

    def getFitness(self, secuenceLength):
        pass

    def executePose(self, pose):
        pass

    def getFrame(self):
        pass

    def stopLucy(self):
        pass

    def isLucyUp(self):
        pass

    def getPosesExecutedByStepQty(self):
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

    def getPosesExecutedByStepQty(self):
        return 1

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

    def getFitness(self, secuenceLength):
        return 0

    def isLucyUp(self):
        return True


#Lucy instanciated in a third party Bioloid Premium robot model
class SimulatedLucy(Lucy):

    def __init__(self, visible=False):
        Lucy.__init__(self)
        self.visible = visible
        genetic_bioloid = os.getcwd() + self.sysConf.getFile("Lucy vrep model")
        self.sim = Simulator(genetic_bioloid)
        self.clientID = self.sim.getClientId() 
        if self.clientID == -1:
            raise VrepException("error connecting with Vrep", -1)
        self.sim.startSim(self.clientID,self.visible)
        self.jointHandleCachePopulated = False
        configuration = LoadRobotConfiguration()
        self.joints = configuration.getJointsName()
        self.startPosSetted = False
        self.firstCallGetFrame = True
        self.angleBetweenOriginAndDestination = []
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
    
    def listAverage(self, l):
        average = sum(l)/len(l) 
        return average

    def listMode(self, l):
        data = Counter(l)
        if len(data) > 0:
            data.most_common()   # Returns all unique items and their counts
            return data.most_common(1)[0][0]  # Returns the highest occurring item
        else:
            return 0

    def getFitness(self, secuenceLength):
        distance = self.getSimDistance()
        error, upD = self.sim.getUpDistance()
        mode = self.listMode(self.angleBetweenOriginAndDestination)
        normMode = mode/180
        #framesQty = int(self.sysConf.getProperty("Individual frames quantity"))
        framesQty = secuenceLength
        #time = self.getSimTime()
        #print "execution time: ", time
        print "--------------------------------------------------------------------"
        print "distance traveled: ", distance
        print "poses executed/total poses: ",  self.poseExecuted, "/", framesQty
        if self.isLucyUp():
            print "isRobotUp?: True"
            framesExecuted = 1
            endCycleBalance = upD/BALANCE_HEIGHT
            if endCycleBalance > 1:
                endCycleBalance = 1
        else:
            print "isRobotUp?: False"
            if framesQty > 0:
                framesExecuted = self.poseExecuted / float(framesQty)
            else:
                framesExecuted = 0
            endCycleBalance = 0
        fitness = 0.30 * distance**(1/4.0) + 0.30 * framesExecuted +  0.4 * endCycleBalance**6
        #fitness = 0.25 * math.sqrt(distance) + 0.3 * framesExecuted + 0.15 * normMode + 0.3 * endCycleBalance**4 evoluciona a estar érgido y caminar moviendo las piernas muy poco
        #fitness = 0.4 * framesExecuted + 0.2 * normMode + 0.4 * endCycleBalance**4 evoluciona a estar érgido y caminar moviendo las piernas muy poco
        print "normMode: ", normMode
        print "framesExecuted: ", framesExecuted
        print "FITNESS: ", fitness
        print "upDistance: ", self.sim.getUpDistance()
        print "endCycleBalance", endCycleBalance
        print "--------------------------------------------------------------------"
        return fitness

    def getPosesExecutedByStepQty(self):
        return self.sim.getPosesExecutedByStepQty(self.clientID)

    def executePose(self, pose):
        error = False
        dontSupportedJoints = self.sysConf.getVrepNotImplementedBioloidJoints()
        RobotImplementedJoints = []
        #Above's N joints will be received and set on the V-REP side at the same time

        #TODO this must be checked in the simulator class
        if (self.jointHandleCachePopulated == False): 
            self.sim.populateJointHandleCache(self.clientID)
            self.jointHandleCachePopulated = True

        for joint in self.joints:
            if joint not in dontSupportedJoints:
                RobotImplementedJoints.append(joint)

        jointsQty = len(RobotImplementedJoints)
        jointExecutedCounter=0

        error = self.sim.pauseSim(self.clientID) or error
        for joint in RobotImplementedJoints:
            angle = pose.getValue(joint) 
            angleAX = AXAngle()   
            angleAX.setDegreeValue(angle)
            #print "setting joint: ", joint, " to value: ", angle

            if jointExecutedCounter < jointsQty - 1:
                error = self.sim.setJointPositionNonBlock(self.clientID, joint, angleAX.toVrep()) or error
            else:
                error = self.sim.resumePauseSim(self.clientID) or error
                error = self.sim.setJointPosition(self.clientID, joint, angleAX.toVrep()) or error

            jointExecutedCounter = jointExecutedCounter + 1

        self.updateLucyPosition()
        self.poseExecuted = self.poseExecuted + self.getPosesExecutedByStepQty()
        #if error:
        #    raise VrepException("error excecuting a pose", error)

    def getFrame(self):
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
                pose[joint] = 150 - value * 60

        self.firstCallGetFrame = False
        error = self.sim.resumePauseSim(self.clientID) or error
        #if error:
        #    raise VrepException("error geting a frame", error)
        return error, pose
    
    def angle(self,v):
        if v.imag >=0:
            resAngle = angle(v, True) #angle second argument is for operate with degrees instead of radians
        else:
            resAngle = 180+angle(-v, True) #angle second argument is for operate with degrees instead of radians
        return resAngle

    def updateLucyPosition(self):
        if self.stop == False: 
            self.time = time.time() - self.startTime
            errorPosition, x, y = self.sim.getBioloidPlannarPosition(self.clientID)
            #print "x_position: ", x, "y_position: ", y 
            if self.startPosSetted and not errorPosition:
                #self.distance = math.sqrt((x-self.startPos[X])**2 + (y-self.startPos[Y])**2)
                #distToGoal = math.sqrt((x-1)**2 + (y-0)**2)
                error, distToGoal = self.sim.getDistanceToSceneGoal()
                distTravelToGoal = 1 - distToGoal
                if distTravelToGoal < 0 :
                    self.distance = 0
                else: 
                    self.distance = distTravelToGoal
            
                #calculates the angle in the frontal plane generated with the vectors j3 to j1 and j2 to j1 in anti clockwise 
                x3 = 1; y3 = 0; z3 = 0;
                x2 = 0; y2 = 0; z2 = 0;
                x1 = x; y1 = y; z1 = 0;   
                u = (x2 - x1) + 1j*(y2 - y1)
                v = (x3 - x1) + 1j*(y3 - y1)
                r = self.angle(u*conjugate(v))
                angle = r.real
                if angle > 180:
                    angle = 360 - angle
                self.angleBetweenOriginAndDestination.append(angle)
                #print "the angle formed by the start point, lucy and destiny is:", angle

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
