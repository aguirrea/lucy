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
from AXAngle import AXAngle

print 'Program started'
clientID = vrep.simxStart('127.0.0.1',19998,True,True,5000,5)
if clientID !=-1:
    print 'Connected to remote API server'
    vrep.simxLoadScene(clientID, "/home/andres/Documentos/maestria/lucy/models/genetic_bioloid.ttt", 2, vrep.simx_opmode_oneshot_wait)
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait)
    if res==vrep.simx_error_noerror:
        print 'Number of objects in the scene: ',len(objs)
    else:
        print 'Remote API function call returned with error code: '
    x =vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    ##vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_display_enabled,0,vrep.simx_opmode_oneshot_wait)
    #error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"LKneePitch3#", vrep.simx_opmode_oneshot_wait)
    #error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"LShoulderPitch3#", vrep.simx_opmode_oneshot_wait)
    error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"L_Shoulder_Pitch#", vrep.simx_opmode_oneshot_wait)
    #                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    error, FLOOR_Handle=vrep.simxGetObjectHandle(clientID,"DefaultFloor#", vrep.simx_opmode_oneshot_wait)
    #vrep.simxCheckDistance(clientID, LSP_Handle, FLOOR_Handle, 50,  vrep.simx_opmode_oneshot_wait)
    #colHandle = vrep.simxGetCollisionHandle(clientID, "DefaultFloor#", vrep.simx_opmode_oneshot_wait)
    #error, collisionState = vrep.simxReadCollision(clientID, colHandle, vrep.simx_opmode_oneshot_wait)
    #print error
    #print collisionState
    angle1 = AXAngle(1023)
    angle2 = AXAngle(512)
    error, position = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
    error, position = vrep.simxGetJointPosition(clientID,LSP_Handle, vrep.simx_opmode_streaming)
    print "al leer posicion error " + str(error) + " posicion " + str(position)

    i=0
    while(1):
        if (i%2 == 0):
            #vrep.simxSetJointPosition(clientID,LSP_Handle, angle1.toVrep() , vrep.simx_opmode_streaming)
            vrep.simxSetJointPosition(clientID,LSP_Handle, 0, vrep.simx_opmode_streaming)
        else:
            vrep.simxSetJointPosition(clientID,LSP_Handle, 1, vrep.simx_opmode_streaming)
        time.sleep(3)
        print "frame: " + str(i)
        error, position = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
        error, position = vrep.simxGetJointPosition(clientID,LSP_Handle, vrep.simx_opmode_streaming)
        print "al leer posicion error " + str(error) + " posicion " + str(position)

        #error, collisionState = vrep.simxReadCollision(clientID, FLOOR_Handle, vrep.simx_opmode_oneshot_wait)
        #print "Error: " + str(error)
        #print "State: " + str(collisionState)
        i=i+1
    x =vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    #vrep.simxFinish()
else:
    print 'Failed connecting to remote API server', clientID
print 'Program ended'


