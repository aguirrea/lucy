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

from LoadPoses import LoadPoses

FALL_THRESHOLD = 0.07
localhost='127.0.0.1'
standadRemoteApiPort=19997


class Simulator:
    def __init__(self):
        #this data structure is like a cache for the joint handles
        self.jointHandleMapping = {}             
        lp = LoadPoses()
        pose=lp.getFramePose(1)
        for joint in pose.keys():
            self.jointHandleMapping[joint]=0

    def connectVREP(self, ipAddr=localhost, port=standadRemoteApiPort):
        vrep.simxFinish(-1) # just in case, close all opened connections
        return vrep.simxStart(ipAddr,port,True,True,5000,5)

    def loadscn(self, clientID, model):
        error=vrep.simxLoadScene(clientID, model, 2, vrep.simx_opmode_oneshot_wait)
        return error

    def printJointPositions(self, clientID):
        error, handles, intData, floatData, stringData=vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_oneshot_wait)
        itemHandle=0
        print stringData
        for name in stringData:
            error, position = vrep.simxGetJointPosition(clientID,handles[itemHandle], vrep.simx_opmode_streaming)
            itemHandle=itemHandle+1
            print name + ":" + str(position)

    def isRobotUp(self, clientID):
        error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"Bioloid", vrep.simx_opmode_oneshot_wait)
        error, bioloid_position = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
        #in case of problems consulting the robot position, consider that the robot is up
        if error:
            return True
        else:
            return bioloid_position[2]>FALL_THRESHOLD

    def startSim(self, clientID, screen=True):
        error=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        if not screen:
            vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_display_enabled,0,vrep.simx_opmode_oneshot_wait)
        return error
        
    def pauseSim(self, clientID):
        return vrep.simxPauseCommunication(clientID,True)
         
    def resumePauseSim(self, clientID):
        return vrep.simxPauseCommunication(clientID,False)

    def populateJointHandleCache(self, clientID):
        lp = LoadPoses()
        pose=lp.getFramePose(1)
        for joint in pose.keys():
            error, handle = vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot_wait)
            self.jointHandleMapping[joint]=handle        
        pass
                
    def setJointPosition(self, clientID, joint, angle):
        if (self.jointHandleMapping[joint] > 0):
            jhandle=self.jointHandleMapping[joint]
        else:
            error, jhandle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot_wait)
            self.jointHandleMapping[joint]=jhandle
        vrep.simxSetJointPosition(clientID,jhandle,angle,vrep.simx_opmode_oneshot_wait)
    
    def setJointPositionNonBlock(self, clientID, joint, angle):
        if (self.jointHandleMapping[joint] > 0):
            jhandle=self.jointHandleMapping[joint]
        else:
            error, jhandle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot)
            self.jointHandleMapping[joint]=jhandle
        vrep.simxSetJointPosition(clientID,jhandle,angle,vrep.simx_opmode_oneshot)
            
    def finishSimulation(self, clientID):
        errorStop=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        errorClose=vrep.simxCloseScene(clientID,vrep.simx_opmode_oneshot_wait)
        error=errorStop or errorClose
        errorFinish=vrep.simxFinish(clientID)
        error=error or errorFinish
        return error
        
    def getBioloidPlannarPosition(self, clientID):
        error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"Bioloid", vrep.simx_opmode_oneshot_wait)
        error, bioloid_position = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
        return bioloid_position[0], bioloid_position[1]



