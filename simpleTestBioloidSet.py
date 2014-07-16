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
import math
from LoadPoses import LoadPoses
from AXAngle import AXAngle

standadRemoteApiPort=19997
localhost='127.0.0.1'
salameIP='192.168.1.101'
windowsSalaIP='192.168.1.106'
genetic_bioloid="/home/andres/Documents/maestria/lucy/models/genetic_bioloid.ttt"
genetic_bioloid_salame="/home/romina/lucy/models/genetic_bioloid.ttt"
genetic_bioloid_windows_sala="C:\genetic_bioloid.ttt"
genetic_bioloid_roca="/home/rfernandez/genetic_bioloid.ttt"
jointHandleMapping = {}

FALL_THRESHOLD = 0.07

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


print 'Program started'
angle = AXAngle()
lp = LoadPoses()
#clientID = connectVREP(windowsSalaIP,standadRemoteApiPort)
clientID = connectVREP()

#pose=lp.getFramePose(1)
#for joint in pose.keys():
#    jointHandleMapping[joint]=0
#    print joint

#clientID = connectVREP(salameIP)
if clientID !=-1:
    print 'Connected to remote API server'
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait)
    print objs
    #loadscn(clientID,genetic_bioloid_salame)
    #loadscn(clientID,genetic_bioloid)
    #loadscn(clientID,genetic_bioloid_windows_sala)
    loadscn(clientID)
    startSim(clientID,True)
    frameQty=lp.getFrameQty()
    #while(1):
    #    for index in range(frameQty):
    #        pose=lp.getFramePose(index)
    #        for joint in pose.keys():
    #            angle.setValue(pose[joint])
    #            setJointPosition(clientID,joint,angle.toVrep())
    #        #printJointPositions(clientID)
    #        print index
    #        #isRobotUp(clientID)
    #for i in range(90): 
    #    value=i+150
    #    print value
    #    angle.setValue(value)
    #    setJointPosition(clientID,"L_Hip_Pitch",angle.toVrep())
    #angle.setDegreeValue(210)
    
    #end=False
    #error=1
    #while not end:
    #    error, up=isRobotUp(clientID)
    #    end=not up
    #print "me cai"
    
    pos1x, pos1y=getBioloidPlannarPosition(clientID)
    for i in range(25):
        angle.setDegreeValue(150-i)
        print "voy a setear ax12 value en cadera izquierda: ", 150 - i
        setJointPosition(clientID,"L_Hip_Pitch",angle.toVrep())
        angle.setDegreeValue(i*2)
        print "voy a setear ax12 value en rodilla derecha: ", i
        setJointPosition(clientID,"R_Knee",angle.toVrep())

    #angle.setDegreeValue(150)
    #setJointPosition(clientID,"R_Hip_Pitch",angle.toVrep())
    print "termine"
    time.sleep(5)
    for i in range(30):
        angle.setDegreeValue(150-i)
        setJointPosition(clientID,"R_Hip_Pitch",angle.toVrep()) 
        setJointPosition(clientID,"L_Knee",angle.toVrep()) 
        isRobotUp(clientID)
        isRobotUp(clientID)
    
    ###angle.setDegreeValue(90)  
    ###setJointPosition(clientID,"R_Knee",angle.toVrep())
    
    time.sleep(5)
    #angle.setValue(180)
    #setJointPosition(clientID,"L_Hip_Pitch",angle.toVrep())
    isRobotUp(clientID)
    pos2x, pos2y=getBioloidPlannarPosition(clientID) 
    print math.sqrt((pos2x-pos1x)**2 + (pos2y-pos1y)**2)   
    error=finishSimulation(clientID)
    print error
else:
    print 'Failed connecting to remote API server', clientID
print 'Program ended'


