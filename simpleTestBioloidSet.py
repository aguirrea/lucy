#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# comunication abstraction layer
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
from LoadPoses import LoadPoses
from AXAngle import AXAngle

standadRemoteApiPort=19998
localhost='127.0.0.1'
salameIP='192.168.1.101'
windowsSalaIP='192.168.1.106'
genetic_bioloid="/home/andres/Documentos/maestria/lucy/models/genetic_bioloid.ttt"
genetic_bioloid_salame="/home/romina/lucy/models/genetic_bioloid.ttt"
genetic_bioloid_windows_sala="C:\genetic_bioloid.ttt"
genetic_bioloid_roca="/home/rfernandez/genetic_bioloid.ttt"

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
    error, position = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)               
    print error, position

def startSim(clientID, screen=True):
    error=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    if not screen:
        vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_display_enabled,0,vrep.simx_opmode_oneshot_wait)
    return error

def setJointPosition(clientID, joint, angle):
    error, jhandle=vrep.simxGetObjectHandle(clientID,joint,vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointPosition(clientID,jhandle,angle,vrep.simx_opmode_oneshot_wait)
        
def finishSimulation(clientID):
    errorStop=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    errorClose=vrep.simxCloseScene(clientID,vrep.simx_opmode_oneshot_wait)
    error=errorStop or errorClose
    errorFinish=vrep.simxFinish(clientID)
    error=error or errorFinish
    return error

print 'Program started'
angle = AXAngle()
lp = LoadPoses()
#clientID = connectVREP(windowsSalaIP,standadRemoteApiPort)
clientID = connectVREP()
#clientID = connectVREP(salameIP)
if clientID !=-1:
    print 'Connected to remote API server'
    #loadscn(clientID,genetic_bioloid_salame)
    #loadscn(clientID,genetic_bioloid)
    #loadscn(clientID,genetic_bioloid_windows_sala)
    loadscn(clientID,genetic_bioloid_windows_sala)
    startSim(clientID)
    frameQty=lp.getFrameQty()
    while(1):
        for index in range(frameQty):
            pose=lp.getFramePose(index)
            for joint in pose.keys():
                angle.setValue(pose[joint])
                setJointPosition(clientID,joint,angle.toVrep())
            #printJointPositions(clientID)
            print index
    error=finishSimulation(clientID)
    print error
else:
    print 'Failed connecting to remote API server', clientID
print 'Program ended'


