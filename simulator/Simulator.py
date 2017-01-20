#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Helper functions to access simulator services
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

from time                    import time

from errors.VrepException    import VrepException

import vrep
from LoadRobotConfiguration  import LoadRobotConfiguration
from LoadSystemConfiguration import LoadSystemConfiguration
from vrepConst import *

bulletEngine = 0
odeEngine    = 1

class Simulator:

    def __init__(self, simulatorModel=None):
        self.robotOrientationFirstTime = True
        self.getDistanceToGoalFirstTime = True
        self.getUpDistanceFirstTime = True
        self.getObjectPositionFirstTime = True
        self.sysConf = LoadSystemConfiguration()
        #this data structure is like a cache for the joint handles
        self.jointHandleMapping = {} 
        robotConf = LoadRobotConfiguration()
        self.model = simulatorModel
        self.LucyJoints = robotConf.getJointsName()            
        for joint in self.LucyJoints:
            self.jointHandleMapping[joint]=0
        self.clientId = self.connectVREP()

        if simulatorModel:
            #TODO try to reutilize the same scene for the sake of performance
            error = self.loadscn(self.clientId, simulatorModel)
            if error:
                 raise VrepException("error loading Vrep robot model", -1)
       
        if int(self.sysConf.getProperty("synchronous mode?"))==1:
            self.synchronous = True
            vrep.simxSynchronousTrigger(self.clientId)
        else:
            self.synchronous = False
        
        self.speedmodifier = int(self.sysConf.getProperty("speedmodifier"))

        #setting the simulation time step                           
        self.simulationTimeStepDT = float(self.sysConf.getProperty("simulation time step"))

        '''#testing printing in vrep
        error, consoleHandler = vrep.simxAuxiliaryConsoleOpen(self.clientId, "lucy debugging", 8, 22, None, None, None, None, vrep.simx_opmode_oneshot_wait)
        print "console handler: ", consoleHandler
        vrep.simxAuxiliaryConsoleShow(self.clientId, consoleHandler, 1, vrep.simx_opmode_oneshot_wait)
        error = vrep.simxAuxiliaryConsolePrint(self.clientId, consoleHandler, "Hello World", vrep.simx_opmode_oneshot_wait)'''

        error, self.upDistanceHandle = vrep.simxGetDistanceHandle(self.clientId, "upDistance#", vrep.simx_opmode_blocking)
        error, self.distLFootToGoalHandle = vrep.simxGetDistanceHandle(self.clientId, "distanceLFootGoal#", vrep.simx_opmode_blocking)
        error, self.distRFootToGoalHandle = vrep.simxGetDistanceHandle(self.clientId, "distanceRFootGoal#", vrep.simx_opmode_blocking)
        error, self.bioloidHandle = vrep.simxGetObjectHandle(self.clientId,"Bioloid", vrep.simx_opmode_oneshot_wait)
        error, self.cuboidHandle = vrep.simxGetObjectHandle(self.clientId,"Cuboid", vrep.simx_opmode_oneshot_wait)
        error, self.armHandler = vrep.simxGetObjectHandle(self.clientId, "Pivot", vrep.simx_opmode_oneshot_wait)

        self.populateJointHandleCache(self.clientId)

        self.isRobotUpFirstCall = True
        self.getDistanceToSceneGoal() #to fix the first invocation
        self.getUpDistance()
        self.isRobotUp(self.clientId)
        self.armPositionXAxis = -9.0000e-02

    def getClientId(self):
        return self.clientId

    def setClientId(self, idClient):
        self.clientId = idClient 

    def getObjectPositionWrapper(self, clientID, LSP_Handle):
        if self.getObjectPositionFirstTime:
            error, ret = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
            self.getObjectPositionFirstTime = False
        else:
            error, ret = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_buffer) 
        return error, ret

    def connectVREP(self, ipAddr=LoadSystemConfiguration.getProperty(LoadSystemConfiguration(),"Vrep IP"), port=int(LoadSystemConfiguration.getProperty(LoadSystemConfiguration(),"Vrep port"))):
        self.getObjectPositionFirstTime = True
        #vrep.simxFinish(-1) # just in case, close all opened connections #TODO this could be a problem when the connection pool is implemented
        time1 = time()
        clientID = vrep.simxStart(ipAddr,port,True,True,5000,5)
        time2 = time()
        print "LOGGING time for connecting VREP in senconds: ", time2 - time1
        return clientID
        
    def loadscn(self, clientID, model):
        time1 = time()
        error = vrep.simxLoadScene(clientID, model, 2, vrep.simx_opmode_oneshot_wait)
        time2 = time()
        print "LOGGING time for loading VREP scene in seconds: ", time2 - time1
        return error

    def printJointPositions(self, clientID):
        error, handlers, intData, floatData, stringData=vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_oneshot_wait)
        itemHandle=0
        print stringData
        for name in stringData:
            error, position = vrep.simxGetJointPosition(clientID,handlers[itemHandle], vrep.simx_opmode_streaming)
            itemHandle=itemHandle+1
            print name + ":" + str(position)

    def isRobotUp(self, clientID):
        GET_DISTANCE_BUGGY_VALUE = 100000
        error, upDistance = self.getUpDistance()
        upDistance = float(upDistance)
        if upDistance > GET_DISTANCE_BUGGY_VALUE: #filter for garbage return on the firsts invocations of vrep.simxReadDistance
            return error, True
        else:
            return error, upDistance > float(self.sysConf.getProperty("FALL_THRESHOLD_DOWN")) and upDistance < float(self.sysConf.getProperty("FALL_THRESHOLD_UP"))

    def startSim(self, clientID, screen=True):
        #I need the simulator stopped in order to be started
        retSimStop = vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        if retSimStop != simx_return_ok :
            print "simulation couldnt be stopped!"
        else:
            print "simulation stopped!"

        #setting the physics engine
        retSetPhyEngine = vrep.simxSetIntegerParameter(clientID, vrep.sim_intparam_dynamic_engine, bulletEngine, vrep.simx_opmode_oneshot_wait)
        if retSetPhyEngine != simx_return_ok:
            print "unable to set the physical engine"
        else:
            print "physical engine correctly setted"

        if int(self.sysConf.getProperty("physics enable?"))==1:
            vrep.simxSetBooleanParameter(clientID,sim_boolparam_dynamics_handling_enabled,True,vrep.simx_opmode_oneshot)
        else:
            vrep.simxSetBooleanParameter(clientID,sim_boolparam_dynamics_handling_enabled,False,vrep.simx_opmode_oneshot)

        #settig simulation speed
        if self.speedmodifier > 0:
            vrep.simxSetIntegerParameter(clientID,vrep.sim_intparam_speedmodifier, self.speedmodifier, vrep.simx_opmode_oneshot_wait)

        #settig simulation step
        retSetTimeStep = vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step, self.simulationTimeStepDT, vrep.simx_opmode_oneshot_wait)
        if retSetTimeStep != simx_return_ok :
            print "problems setting time step"
        else:
            print "time step setted!"

        #vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_realtime_simulation,1,vrep.simx_opmode_oneshot_wait)

        #sync mode configuration
        if self.synchronous:
            vrep.simxSynchronous(clientID,True)

        #light mode configuration
        if not screen:
            vrep.simxSetIntegerParameter(clientID,vrep.sim_intparam_visible_layers,2,vrep.simx_opmode_oneshot_wait)
            #vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_display_enabled,0,vrep.simx_opmode_oneshot_wait)


        #start simulation
        error=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
        if int(self.sysConf.getProperty("blank screen?"))==1:
            vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_display_enabled,0,vrep.simx_opmode_oneshot_wait)
        return error
        
    def pauseSim(self, clientID):
        return vrep.simxPauseCommunication(clientID,True)
         
    def resumePauseSim(self, clientID):
        ret=vrep.simxPauseCommunication(clientID,False)
        if self.speedmodifier > 0:
            vrep.simxSetIntegerParameter(clientID,vrep.sim_intparam_speedmodifier, self.speedmodifier, vrep.simx_opmode_oneshot_wait) 
        return ret

    #when the simulator is paused the call to simxGetObjectHandle returns error    
    def populateJointHandleCache(self, clientID):
        for joint in self.LucyJoints:
            error, handle = vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot_wait)
            self.jointHandleMapping[joint]=handle 
        pass
                
    def setJointPosition(self, clientID, joint, angle):
        error = False
        handle = self.jointHandleMapping[joint]
        if handle == 0:
            errorGetObjetHandle, handle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot_wait)
            error = errorGetObjetHandle or error
            self.jointHandleMapping[joint]=handle
        #error = error or vrep.simxSetJointPosition(clientID,handle,angle,vrep.simx_opmode_oneshot_wait)
        error = error or vrep.simxSetJointPosition(clientID,handle,angle,vrep.simx_opmode_oneshot) #using oneshot instead of oneshot_wait make it a nonblocking call but is the only way to avoid 5 seconds wait in the execution of the individual
        if self.synchronous:
            vrep.simxSynchronousTrigger(clientID)
        return error

    def setJointPositionNonBlock(self, clientID, joint, angle):
        handle = self.jointHandleMapping[joint]
        error = False
        if (handle == 0):
            error, handle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot)
            self.jointHandleMapping[joint]=handle
        return vrep.simxSetJointPosition(clientID,handle,angle,vrep.simx_opmode_streaming)

    def getJointPositionNonBlock(self, clientID, joint, firstTime):
        error = False
        value = 0
        handle = self.jointHandleMapping[joint]
        if handle == 0:
            error, handle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot)
            self.jointHandleMapping[joint]=handle
        if not error:
            if firstTime:
                error, value = vrep.simxGetJointPosition(clientID,handle,vrep.simx_opmode_streaming)
            else:
                error, value = vrep.simxGetJointPosition(clientID,handle,vrep.simx_opmode_buffer)
        return error, value

    def finishSimulation(self, clientID):
        self.getObjectPositionFirstTime = True
        errorStop=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        errorClose=vrep.simxCloseScene(clientID,vrep.simx_opmode_oneshot_wait)
        error=errorStop or errorClose
        errorFinish=vrep.simxFinish(clientID)
        error=error or errorFinish
        if self.synchronous:
            vrep.simxSynchronous(clientID,False)
        return error
        
    def getBioloidPlannarPosition(self, clientID):
        errorHandler, LSP_Handle=vrep.simxGetObjectHandle(clientID,"Bioloid", vrep.simx_opmode_oneshot_wait)
        errorObjectPosition = True
        if not errorHandler:
            errorObjectPosition, bioloid_position = self.getObjectPositionWrapper(clientID, LSP_Handle)
            #print "error handler: ", errorHandler, " error object position: ", errorObjectPosition 
            if not errorHandler or errorObjectPosition:
                return False, bioloid_position[0], bioloid_position[1]
        return True, None, None

    def getPosesExecutedByStepQty(self, clientID):
        return int(float(vrep.simxGetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, vrep.simx_opmode_oneshot_wait)[1] ) / float(0.050))

    def getDistanceToSceneGoal(self):
        if self.getDistanceToGoalFirstTime:
            errorL, distanceL = vrep.simxReadDistance(self.clientId, self.distLFootToGoalHandle, vrep.simx_opmode_streaming)
            errorR, distanceR = vrep.simxReadDistance(self.clientId, self.distRFootToGoalHandle, vrep.simx_opmode_streaming)
            self.getDistanceToGoalFirstTime = False
        else:
            errorL, distanceL = vrep.simxReadDistance(self.clientId, self.distLFootToGoalHandle, vrep.simx_opmode_buffer)
            errorR, distanceR = vrep.simxReadDistance(self.clientId, self.distRFootToGoalHandle, vrep.simx_opmode_buffer)
        return errorL or errorR, max(distanceL, distanceR)

    def getUpDistance(self):
        if self.getUpDistanceFirstTime:
            error, distance = vrep.simxReadDistance(self.clientId, self.upDistanceHandle, vrep.simx_opmode_streaming)
            retrys = 0
            while error and retrys < 1000:
                error, distance = vrep.simxReadDistance(self.clientId, self.upDistanceHandle, vrep.simx_opmode_streaming)
                retrys += 1
            self.getUpDistanceFirstTime = False
        else:
            error, distance = vrep.simxReadDistance(self.clientId, self.upDistanceHandle, vrep.simx_opmode_buffer)
        return error, distance

    def robotOrientationToGoal(self):
        if self.robotOrientationFirstTime:
            error, angle = vrep.simxGetObjectOrientation(self.clientId, self.bioloidHandle, self.cuboidHandle, vrep.simx_opmode_streaming)
            self.robotOrientationFirstTime = False
        else:
            error, angle = vrep.simxGetObjectOrientation(self.clientId, self.bioloidHandle, self.cuboidHandle, vrep.simx_opmode_buffer)
        return error, angle[1]

    def moveHelperArm(self, distTraveled):
        SET_OBJECT_POSITION_ERROR_THRESHOLD = 2
        MOVE_HELPER_ARM_ERROR = 1
        newArmPositionX = float(distTraveled * 1.2)
        newArmPosition = (newArmPositionX, 0, 0)
        #vrep.simxSetBooleanParameter(self.clientId,sim_boolparam_dynamics_handling_enabled,False,vrep.simx_opmode_oneshot)
        #moves relative to the robot itself
        error = vrep.simxSetObjectPosition(self.clientId, self.armHandler, self.armHandler, newArmPosition, vrep.simx_opmode_oneshot)
        #vrep.simxSetBooleanParameter(self.clientId,sim_boolparam_dynamics_handling_enabled,True,vrep.simx_opmode_oneshot)
        if error < SET_OBJECT_POSITION_ERROR_THRESHOLD:
            self.armPositionXAxis = newArmPosition
        else:
            return MOVE_HELPER_ARM_ERROR

        







