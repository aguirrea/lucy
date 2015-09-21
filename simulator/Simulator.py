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

import vrep
import time
import math

from LoadRobotConfiguration import LoadRobotConfiguration
from LoadSystemConfiguration import LoadSystemConfiguration

bulletEngine = 0
odeEngine    = 1
simulationTimeStepDT = 0.020
_instance    = None
class Simulator:

    def __init__(self, simulatorModel=None):
        self.getObjectPositionFirstTime = True
        #this data structure is like a cache for the joint handles
        self.jointHandleMapping = {} 
        robotConf = LoadRobotConfiguration()
        self.model = simulatorModel
        self.LucyJoints = robotConf.getJointsName()            
        for joint in self.LucyJoints:
            self.jointHandleMapping[joint]=0
        self.clientId = self.connectVREP()
        if simulatorModel:
            self.loadscn(self.clientId, simulatorModel)

    def getInstance(self, simulatorModel):
        global _instance
        if _instance is None:
            _instance = Simulator(simulatorModel)
        return _instance
    
    def getClientId(self):
        return self.clientId

    def setClientId(self, idClient):
        self.clientId = idClient 

    def getObjectPositionWrapper(self, clientID, LSP_Handle):
        error = False
        if self.getObjectPositionFirstTime:
            error, ret = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
            self.getObjectPositionFirstTime = False
        else:
            error, ret = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_buffer) 
        return error, ret

    def connectVREP(self, ipAddr=LoadSystemConfiguration.getProperty(LoadSystemConfiguration(),"Vrep IP"), port=int(LoadSystemConfiguration.getProperty(LoadSystemConfiguration(),"Vrep port"))):
        self.getObjectPositionFirstTime = True
        vrep.simxFinish(-1) # just in case, close all opened connections
        return vrep.simxStart(ipAddr,port,True,True,5000,5)

    def loadscn(self, clientID, model):
        error=vrep.simxLoadScene(clientID, model, 2, vrep.simx_opmode_oneshot_wait)
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
        error = False
        error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"Bioloid", vrep.simx_opmode_oneshot_wait) or error
        error, bioloid_position = self.getObjectPositionWrapper(clientID, LSP_Handle) or error 
        return error, bioloid_position[2]>float(LoadSystemConfiguration.getProperty(LoadSystemConfiguration(),"FALL_THRESHOLD"))

    def startSim(self, clientID, screen=True):
        vrep.simxSetIntegerParameter(clientID,vrep.sim_intparam_dynamic_engine,bulletEngine ,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step, simulationTimeStepDT, vrep.simx_opmode_oneshot_wait)
        #vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_realtime_simulation,1,vrep.simx_opmode_oneshot_wait)
        error=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
        if not screen:
            vrep.simxSetIntegerParameter(clientID,vrep.sim_intparam_visible_layers,2,vrep.simx_opmode_oneshot_wait)
            #vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_display_enabled,0,vrep.simx_opmode_oneshot_wait)
        vrep.simxSynchronous(clientID,True)
        return error
        
    def pauseSim(self, clientID):
        return vrep.simxPauseCommunication(clientID,True)
         
    def resumePauseSim(self, clientID):
        return vrep.simxPauseCommunication(clientID,False)

    #when the simulator is paused the call to simxGetObjectHandle returns error    
    def populateJointHandleCache(self, clientID):
        for joint in self.LucyJoints:
            error, handle = vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot_wait)
            self.jointHandleMapping[joint]=handle 
        pass
                
    def setJointPosition(self, clientID, joint, angle):
        error = False
        handle = self.jointHandleMapping[joint]
        if (handle == 0):
            errorGetObjetHandle, handle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot_wait)
            error = errorGetObjetHandle or error
            self.jointHandleMapping[joint]=handle
        error = error or vrep.simxSetJointPosition(clientID,handle,angle,vrep.simx_opmode_oneshot_wait)
        vrep.simxSynchronousTrigger(clientID)
        return error
    
    def setJointPositionNonBlock(self, clientID, joint, angle):
        handle = self.jointHandleMapping[joint]
        error = False
        if (handle == 0):
            error, handle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot)
            self.jointHandleMapping[joint]=handle
        vrep.simxSetJointPosition(clientID,handle,angle,vrep.simx_opmode_streaming)
        vrep.simxSynchronousTrigger(clientID)
        #return error TODO?
        
    def getJointPositionNonBlock(self, clientID, joint, firstTime):
        error = False
        value = 0
        handle = self.jointHandleMapping[joint]
        if (handle == 0):
            error, handle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot)
            self.jointHandleMapping[joint]=handle
        if not error:
            '''while we are connected:'''
            while vrep.simxGetConnectionId(clientID) != -1:
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






