#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# helper functions to access to the simulator
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

FALL_THRESHOLD = 0.07

#this data structure is like a cache for the joint handles
jointHandleMapping = {}

def connectVREP(ipAddr=localhost, port=standadRemoteApiPort):
    return vrep.simxStart(ipAddr,port,True,True,5000,5)

def loadscn(clientID, model=genetic_bioloid):
    error=vrep.simxLoadScene(clientID, model, 2, vrep.simx_opmode_oneshot_wait)
    return error

def printJointPositions(clientID):
    error, handles, intData, floatData, stringData=vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_oneshot_wait)
    itemHandle=0
    print stringData
    for name in stringData:
        error, position = vrep.simxGetJointPosition(clientID,handles[itemHandle], vrep.simx_opmode_streaming)
        itemHandle=itemHandle+1
        print name + ":" + str(position)

def isRobotUp(clientID):
    error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"Bioloid", vrep.simx_opmode_oneshot_wait)
    error, bioloid_position = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
    #in case of problems consulting the robot position, consider that the robot is up
    if error:
        return error, True
    else:
        return error, bioloid_position[2]>FALL_THRESHOLD

def startSim(clientID, screen=True):
    error=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    if not screen:
        vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_display_enabled,0,vrep.simx_opmode_oneshot_wait)
    return error

def setJointPosition(clientID, joint, angle):
    if (jointHandleMapping[joint] > 0):
        jhandle=jointHandleMapping[joint]
    else:
        error, jhandle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot_wait)
        jointHandleMapping[joint]=jhandle
    vrep.simxSetJointPosition(clientID,jhandle,angle,vrep.simx_opmode_oneshot_wait)
        
def finishSimulation(clientID):
    errorStop=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    errorClose=vrep.simxCloseScene(clientID,vrep.simx_opmode_oneshot_wait)
    error=errorStop or errorClose
    errorFinish=vrep.simxFinish(clientID)
    error=error or errorFinish
    return error
    
def getBioloidPlannarPosition(clientID):
    error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"Bioloid", vrep.simx_opmode_oneshot_wait)
    error, bioloid_position = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
    return bioloid_position[0], bioloid_position[1]


