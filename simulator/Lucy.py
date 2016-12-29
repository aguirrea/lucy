#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Lucy robot software representation.
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
import time
from numpy import angle

from datatypes.DTFitness            import DTFitness
from errors.VrepException           import VrepException

from AXAngle                        import AXAngle
from Actuator                       import Actuator
from Communication                  import CommSerial
from FitnessFunctionFactory         import DistanceConcatenationgapFramesexecutedEndcyclebalanceAngle
from LoadRobotConfiguration         import LoadRobotConfiguration
from LoadSystemConfiguration        import LoadSystemConfiguration
from Simulator                      import Simulator

#BALANCE_HEIGHT = 0.214 #Distance from the floor when lucy is straight up; classical model
BALANCE_HEIGHT = 0.325 #Distance from the floor when lucy is straight up; Björn P Mattsson model

#abstract class representing lucy abstraction base class
class Lucy(object):
    def __init__(self):
        self.sysConf = LoadSystemConfiguration()
        self.robotConfiguration = LoadRobotConfiguration()
        self.joints = self.robotConfiguration.getJointsName()
        self.time = 0
        self.startTime = time.time()
        self.distance = 0
        self.distanceBeforMoveArmLastCall = 0
        self.armDistance = 0
        self.stop = False
        self.poseExecuted = 0
        dontSupportedJoints = self.sysConf.getVrepNotImplementedBioloidJoints()
        self.RobotImplementedJoints = []
        for joint in self.joints:
            if joint not in dontSupportedJoints:
                self.RobotImplementedJoints.append(joint)

    def getFitness(self, secuenceLength, concatenationGap):
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
        self.actuator = Actuator(self.comm_tty)
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
        #TODO tener en cuenta los motores que están invertidos, creo que los únicos que están quedando son los de los hombros

        for joint in self.RobotImplementedJoints:
            angle = pose.getValue(joint)
            angleAX = AXAngle()
            angleAX.setDegreeValue(angle)
            #TODO implement method for setting position of all actuators at the same time
            self.actuator.move_actuator(self.robotConfiguration.loadJointId(joint), int(angleAX.getValue()), self.defaultSpeed)
            self.poseExecuted = self.poseExecuted + 1
        print "pose executed!"
        time.sleep(0.1)

    def stopLucy(self):
        pass
        '''
        for joint in self.joints:
            self.actuator.move_actuator(self.robotConfiguration.loadJointId(joint), self.initialPoses[joint], self.defaultSpeed)
        '''

    def getFrame(self):
        error = False
        pose = {}
        for joint in self.joints:
            value = self.actuator.get_position(self.robotConfiguration.loadJointId(joint))
            pose[joint] = value
        return error, pose

    def getFitness(self, secuenceLength, concatenationGap):
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
            retry_counter = 1000
            time.sleep(0.1)
            self.clientID = self.sim.getClientId()
            while self.clientID == -1 and retry_counter > 0:
                time.sleep(0.5)
                retry_counter = retry_counter - 1
                self.clientID = self.sim.getClientId()
                print "waiting for vrep connection"
            if self.clientID == -1:
                raise VrepException("error connecting with Vrep", -1)
        self.sim.startSim(self.clientID,self.visible)
        self.jointHandleCachePopulated = False
        configuration = LoadRobotConfiguration()
        self.joints = configuration.getJointsName()
        self.firstCallGetFrame = True
        self.sim.robotOrientationToGoal()
        self.updateLucyPosition()
        self.posesExecutedByStepQty = self.sim.getPosesExecutedByStepQty(self.clientID)

    def getSimTime(self):
        if self.stop == False:
            self.time = time.time() - self.startTime
        return self.time
        
    def getSimDistance(self):
        return self.distance

    def getFitness(self, secuenceLength, concatenationGap):
        error, angle = self.sim.robotOrientationToGoal()
        distance = self.getSimDistance()
        error, upD = self.sim.getUpDistance()
        framesQty = secuenceLength
        cycleEnded = 0

        print "--------------------------------------------------------------------"
        print "orientation: ", angle
        print "distance traveled: ", distance
        print "poses executed/total poses: ",  self.poseExecuted, "/", framesQty
        if self.isLucyUp():
            print "isRobotUp?: True"
            cycleEnded = 1
            framesExecuted = 1
            endCycleBalance = upD/BALANCE_HEIGHT
            if endCycleBalance > 1:
                endCycleBalance = 1
        else:
            print "isRobotUp?: False"
            if framesQty > 0:
                framesExecuted = self.poseExecuted / float(framesQty)
                if framesExecuted == 1:
                    framesExecuted = framesExecuted - framesExecuted/10
            else:
                framesExecuted = 0
            endCycleBalance = 0

        dtFitness = DTFitness(distance, concatenationGap, framesExecuted, endCycleBalance, angle)
        fitnessFunction = DistanceConcatenationgapFramesexecutedEndcyclebalanceAngle(dtFitness)


        #dtFitness = DTFitness(distance=distance, concatenationGap=concatenationGap, framesExecuted=framesExecuted, angle=angle, cycleEnded=cycleEnded)
        #fitnessFunction = NormdistanceConcatenationgapFramesexecutedNormAngle(dtFitness)
        #fitnessFunction = ConcatenationgapFramesexecutedNormAngle(dtFitness)
        fitness = fitnessFunction.getFitness()

        print "framesExecuted: ", framesExecuted
        print "FITNESS: ", fitness
        print "upDistance: ", self.sim.getUpDistance()
        print "endCycleBalance: ", endCycleBalance

        print "--------------------------------------------------------------------"
        return fitness

    def getPosesExecutedByStepQty(self):
        return self.posesExecutedByStepQty

    def executePose(self, pose):
        error = False
        #Above's N joints will be received and set on the V-REP side at the same time

        jointsQty = len(self.RobotImplementedJoints)
        jointExecutedCounter = 0

        error = self.sim.pauseSim(self.clientID) or error
        for joint in self.RobotImplementedJoints:
            angle = pose.getValue(joint) 
            angleAX = AXAngle()   
            angleAX.setDegreeValue(angle)
            #print "setting joint: ", joint, " to value: ", angle

            if jointExecutedCounter < jointsQty - 1:
                error = self.sim.setJointPositionNonBlock(self.clientID, joint, angleAX.toVrep()) or error
            else:
                error = self.sim.resumePauseSim(self.clientID) or error
                error = self.sim.setJointPosition(self.clientID, joint, angleAX.toVrep()) or error

            jointExecutedCounter += 1
            self.updateLucyPosition()

        self.poseExecuted = self.poseExecuted + self.getPosesExecutedByStepQty()
        #if error:
        #    raise VrepException("error excecuting a pose", error)

    def executeRawPose(self, pose):
        error = False
        #Above's N joints will be received and set on the V-REP side at the same time

        jointsQty = len(self.RobotImplementedJoints)
        jointExecutedCounter=0

        error = self.sim.pauseSim(self.clientID) or error
        for joint in self.RobotImplementedJoints:
            angle = pose.getValue(joint)
            #print "setting joint: ", joint, " to value: ", angle

            if jointExecutedCounter < jointsQty - 1:
                error = self.sim.setJointPositionNonBlock(self.clientID, joint, angle) or error
            else:
                error = self.sim.resumePauseSim(self.clientID) or error
                error = self.sim.setJointPosition(self.clientID, joint, angle) or error

            jointExecutedCounter += 1
            self.updateLucyPosition()
            print "distance traveled: ", self.distance
        self.poseExecuted = self.poseExecuted + self.getPosesExecutedByStepQty()

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
            error, distToGoal = self.sim.getDistanceToSceneGoal()
            distTravelToGoal = 1.0 - distToGoal
            if distTravelToGoal < 0:
                self.distance = 0
            else:
                self.distance = distTravelToGoal

    def stopLucy(self):
        self.stop = True
        self.updateLucyPosition()
        self.sim.finishSimulation(self.clientID)
            
    def isLucyUp(self):
        error, up = self.sim.isRobotUp(self.clientID)
        if error:
            #raise VrepException("error consulting if lucy is up", error)
            return False
        return up

    def moveHelperArm(self):
        firstTime = self.armDistance == 0
        armDistanceStep = self.distance - self.distanceBeforMoveArmLastCall
        #armDistanceStep = self.distance - self.armDistance #esto tiene que ser el step que hizo el robot
        if armDistanceStep > 0:
            self.armDistance = self.armDistance + armDistanceStep
        error = self.sim.moveHelperArm(armDistanceStep, firstTime)
        self.distanceBeforMoveArmLastCall = self.distance
        print "distancia que muevo el brazo: ", armDistanceStep
        return error


