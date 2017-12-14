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
import datetime
from numpy import angle

from datatypes.DTFitness            import DTFitness
from errors.VrepException           import VrepException

import configuration.constants as sysConstants
from AXAngle                        import AXAngle
from Actuator                       import Actuator
#from DynamixelActuator              import DynamixelActuator
from Communication                  import CommSerial
import Communication
from FitnessFunctionFactory         import DistanceConcatenationgapFramesexecutedEndcyclebalanceAngle
from LoadRobotConfiguration         import LoadRobotConfiguration
from LoadSystemConfiguration        import LoadSystemConfiguration
from Simulator                      import Simulator
from pydynamixel import dynamixel, chain

MAX_SPEED = 500
DEFAULT_SPEED = 250
MIN_SPEED = 10

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

        self.balanceHeight = float(self.sysConf.getProperty("BALANCE_HEIGHT"))

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

    def setCommunication(self, communication):
        pass

#Lucy instanciated in a Bioloid Premium robot
class PhysicalLucy(Lucy):

    def __init__(self):

        Lucy.__init__(self)

        #self.comm_tty = CommSerial()
        #self.comm_tty.connect()

        self.ser = Communication.get_serial_for_url('/dev/tty.usbserial-A900fDga')
        #self.ser = Communication.get_serial_for_url('/dev/tty.usbserial-A7005LBF')


        self.actuator = Actuator(self.ser)
        #self.actuator = Actuator(self.comm_tty)

        #self.defaultSpeed = 500 #TODO change this, use configuration files
        self.initialPoses = {}

        self.currentAngle = {}
        self.targetAngle = {}
        self.max_distance_joint = -1

        self.syncSpeeds = {}
        self.syncPositions = {}

        self.minAngle = {1:0 , 2:0 , 3:45 , 4:55 , 5:46 , 6:50 , 7:140 , 8:110 , 9:110 , 10:140 , 11:35 , 12:125 , 13:55 , 14:145 , 15:125 , 16:50 , 17:110 , 18:100}
        self.maxAngle = {1:300 , 2:300 , 3:245 , 4:255 , 5:250 , 6:254 , 7:190 , 8:160 , 9:160 , 10:190 , 11:175 , 12:265 , 13:155 , 14:245 , 15:250 , 16:175 , 17:200 , 18:190}

        #self.ser = dynamixel.get_serial_for_url('/dev/tty.usbserial-A900fDga')
        self.testpos = []
        self.testvel = []
        self.testservos = []
        '''
        #checking communication with motors
        for joint in self.joints:
            jointID = self.robotConfiguration.loadJointId(joint)

            print 'joint id: ' + str(jointID)
            self.initialPoses[joint] = self.actuator.get_position(jointID).toDegrees()
            print jointID, joint, self.initialPoses[joint]
            time.sleep(0.1)
            self.actuator.led_state_change(jointID, 1)
            time.sleep(0.1)

        time.sleep(1)

        for joint in self.joints:
            self.actuator.led_state_change(self.robotConfiguration.loadJointId(joint), 0)
        '''

    '''
    def setCommunication(self, communication):

        self.comm_tty = communication
        self.actuator = Actuator(self.comm_tty)

    '''

    def getPosesExecutedByStepQty(self):
        return 1


    def findMaxDistance(self, pose):

        #Algorithm for setting speed and angles for each motor
        max_distance = 0
        max_distance_joint = -1

        for joint in self.RobotImplementedJoints:
            target_angle = pose.getValue(joint)

            if (joint == "R_Ankle_Pitch" or joint == "R_Knee" or
                joint == "L_Hip_Pitch" or joint == "L_Shoulder_Pitch" ):
                target_angle = 300 - target_angle
            else:
                target_angle = target_angle

            if target_angle > 300:
                print "Invalid angle " + str(target_angle)

            jointID = self.robotConfiguration.loadJointId(joint)
            #self.currentAngle[joint] = self.actuator.get_position(jointID).toDegrees()
            distance = abs(target_angle - self.currentAngle[joint])
            if distance > max_distance :
                max_distance = distance
                max_distance_joint = jointID

        return max_distance_joint, max_distance


    def waitForCompletion(self, pose, timeout):

        margin = 2
        ended = False;
        start_time = datetime.datetime.now()

        while (not ended):

            '''
            for joint in self.RobotImplementedJoints:
                jointID = self.robotConfiguration.loadJointId(joint)
                self.currentAngle[joint] = self.actuator.get_position(jointID).toDegrees()
                time.sleep(0.01)
            '''
            real_angle = self.actuator.get_position(self.max_distance_joint).toDegrees()

            ended = True
            #for joint in self.RobotImplementedJoints:
            #if (not ((self.targetAngle[joint] - margin <= self.currentAngle[joint]) and (self.currentAngle[joint] < self.targetAngle[joint] + margin)) ):
            if (not ((self.targetAngle[self.max_distance_joint] - margin <= real_angle) and (real_angle < self.targetAngle[self.max_distance_joint] + margin)) ):
                ended = False
                print ("False")
                print "Real angle: ", real_angle
                time.sleep(0.01)

            '''
            if ( (datetime.datetime.now()-start_time).microseconds >= timeout):
                print (str(datetime.datetime.now()-start_time).microseconds))
                print ("Timeout!")
                ended = True;
            '''

        print ("llego el motor")


    def idlePosition(self):

        angleAX = AXAngle()

        #Seteo brazos
        angleAX.setDegreeValue(230)
        time.sleep(0.01)
        #self.actuator.move_actuator(4, int(angleAX.getValue()), DEFAULT_SPEED)
        self.currentAngle[4] = 230
        angleAX.setDegreeValue(70)
        time.sleep(0.01)
        #self.actuator.move_actuator(3, int(angleAX.getValue()), DEFAULT_SPEED)
        self.currentAngle[3] = 70

        angleAX.setDegreeValue(150)
        time.sleep(0.01)
        #self.actuator.move_actuator(5, int(angleAX.getValue()), DEFAULT_SPEED)
        self.currentAngle[5] = 150
        angleAX.setDegreeValue(150)
        time.sleep(0.01)
        #self.actuator.move_actuator(6, int(angleAX.getValue()), DEFAULT_SPEED)
        self.currentAngle[6] = 150

        #Seteo resto del cuerpo

        for joint in self.RobotImplementedJoints:

            jointID = self.robotConfiguration.loadJointId(joint)

            if joint == 'L_Shoulder_Pitch':
                self.syncPositions[jointID] = 230
            elif joint == 'R_Shoulder_Pitch':
                self.syncPositions[jointID] = 70
            else:
                self.syncPositions[jointID] = 150

            self.syncSpeeds[jointID] = DEFAULT_SPEED

            #real_angle = self.actuator.get_position(jointID).toDegrees()
            self.currentAngle[joint] = 150


        time.sleep(0.01)
        #self.actuator.sync_move(self.syncPositions, self.syncSpeeds)
        time.sleep(0.01)

        '''
        jointID = 2
        speed = 100
        angleAX = AXAngle()
        angleAX.setDegreeValue(150)
        print (self.actuator.get_position(jointID).toDegrees())
        self.actuator.move_actuator(jointID, int(angleAX.getValue()), speed)
        '''


    def executePose(self, pose):

        start_pose = time.time()

        #set positions and wait that the actuator reaching that position

        #max_speed = 500
        #min_speed = 10
        timeout = 200

        #start = time.time()
        #max_distance = 150
        self.max_distance_joint, max_distance = self.findMaxDistance(pose)

        print "-------------------------"
        print "Max distance: " + str(max_distance)
        print "Max distance jointID: " + str(self.max_distance_joint)


        #end = time.time()
        #print (end-start)

        '''
        jointIDs = []
        for joint in self.RobotImplementedJoints:
            jointID = self.robotConfiguration.loadJointId(joint)
            jointIDs.append(jointID)
            #self.currentAngle[joint] = 150

            self.currentAngle[joint] = self.actuator.get_position(jointID).toDegrees()
            print self.currentAngle[joint]
            time.sleep(0.01)
        '''

        for joint in self.RobotImplementedJoints:

            #start = time.time()
            jointID = self.robotConfiguration.loadJointId(joint)
            #print "Joint: ", jointID

            target_angle = pose.getValue(joint)
            #print "Target Angle: ", target_angle

            '''
            if target_angle > 300:
                target_angle = 300
            '''

            #end = time.time()
            #print (end-start)

            if (joint == "R_Ankle_Pitch" or joint == "R_Knee" or
                joint == "L_Hip_Pitch" or joint == "L_Shoulder_Pitch" ):
                angle = int(300 - target_angle)
            else:
                angle = int(target_angle)

            #correct angle
            if angle < self.minAngle[jointID]:
                print (str(jointID) + ': Angle out of min range - ' + str(self.minAngle[jointID]-angle) )
                angle = self.minAngle[jointID]

            elif angle > self.maxAngle[jointID]:
                print (str(jointID) + ': Angle out of max range - ' + str(angle-self.maxAngle[jointID]) )
                angle = self.maxAngle[jointID]

            #print "Fixed Angle: ", angle

            distance = abs(angle - self.currentAngle[joint])
            speed = DEFAULT_SPEED
            speed = int((distance * MAX_SPEED) / max_distance)

            #print "Distance: ", distance

            #speed = 300
            if speed > MAX_SPEED:
                speed = MAX_SPEED
            elif speed < MIN_SPEED:
                speed = MIN_SPEED

            #print "Speed: ", speed

            angleAX = AXAngle()
            angleAX.setDegreeValue(angle)

            self.targetAngle[jointID] = angle

            self.syncPositions[jointID] = angle
            self.syncSpeeds[jointID] = speed

            #self.testservos.append(jointID)
            #self.testpos.append(int(angleAX.getValue()))
            #self.testvel.append(speed)

            #self.actuator.move_actuator(jointID, int(angleAX.getValue()), speed)
            #time.sleep(0.01)

        self.actuator.sync_move(self.syncPositions, self.syncSpeeds)

        time.sleep(0.04)

        self.poseExecuted = self.poseExecuted + 1

        print "pose executed: ", str(self.poseExecuted)
        end_pose = time.time()
        print "Pose execution time: "
        print (end_pose - start_pose)

        #self.waitForCompletion(pose, timeout)

        for joint in self.RobotImplementedJoints:
            jointID = self.robotConfiguration.loadJointId(joint)
            self.currentAngle[joint] = self.targetAngle[jointID]

        '''
        for joint in self.RobotImplementedJoints:
            print "el joint se reporta como: ", joint
            angle = pose.getValue(joint)
            angleAX = AXAngle()
            angleAX.setDegreeValue(angle)
            self.actuator.move_actuator(self.robotConfiguration.loadJointId(joint), int(angleAX.getValue()), self.defaultSpeed)
            time.sleep(0.03)
        self.poseExecuted = self.poseExecuted + 1
        print "pose executed!"
        time.sleep(0.04)
        '''

    def stopLucy(self):
        #pass
        Communication.close_serial(self.ser)
        #self.comm_tty.flushInput()
        #self.comm_tty.flushOutput()
        #self.comm_tty.close()
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
            retry_counter = sysConstants.ERROR_RETRY
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
            endCycleBalance = upD/self.balanceHeight #Distance from the floor when lucy is straight up
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
                ###self.moveHelperArm()
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
            if not error:
                distTravelToGoal = 1.0 - distToGoal
                if distTravelToGoal < 0:
                    self.distance = 0
                else:
                    self.distance = distTravelToGoal
            else:
                print "*****************************************ERROR CALCULATING DIST TO GOAL IN Lucy:updateLucyPosition"

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
        error = False
        armDistanceStep = self.distance - self.distanceBeforMoveArmLastCall
        if armDistanceStep > 0:
            self.armDistance = self.armDistance + armDistanceStep
            #error = self.sim.pauseSim(self.clientID) or error
            error = self.sim.moveHelperArm(armDistanceStep) or error
            #error = self.sim.resumePauseSim(self.clientID) or error
            self.distanceBeforMoveArmLastCall = self.distance
        return error
