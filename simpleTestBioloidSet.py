# Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# -------------------------------------------------------------------
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# 
# You are free to use/modify/distribute this file for whatever purpose!
# -------------------------------------------------------------------

# This file was automatically created for V-REP release V3.0.1 on January 20th 2013

# Make sure to have the server side running in V-REP!
# Start the server from a child script with following command:
# simExtRemoteApiStart(19999) -- starts a remote API server service on port 19999

import vrep
import time

print 'Program started'
clientID = vrep.simxStart('127.0.0.1',19998,True,True,5000,5)
if clientID !=-1:
    print 'Connected to remote API server'
    ##vrep.simxLoadScene(clientID, "/home/andres/Documentos/maestria/modelos_fisicos_robots/nao_genetico.ttt", 2, vrep.simx_opmode_oneshot_wait)
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_oneshot_wait)
    if res==vrep.simx_error_noerror:
        print 'Number of objects in the scene: ',len(objs)
    else:
        print 'Remote API function call returned with error code: '
    ##x =vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    ##vrep.simxSetBooleanParameter(clientID,vrep.sim_boolparam_display_enabled,0,vrep.simx_opmode_oneshot_wait)
    error, LSP_Handle=vrep.simxGetObjectHandle(clientID,"LShoulderPitch3#", vrep.simx_opmode_oneshot_wait)
    print error
    print LSP_Handle
    #                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    error, FLOOR_Handle=vrep.simxGetObjectHandle(clientID,"DefaultFloor#", vrep.simx_opmode_oneshot_wait)
    #vrep.simxCheckDistance(clientID, LSP_Handle, FLOOR_Handle, 50,  vrep.simx_opmode_oneshot_wait)
    #colHandle = vrep.simxGetCollisionHandle(clientID, "DefaultFloor#", vrep.simx_opmode_oneshot_wait)
    #error, collisionState = vrep.simxReadCollision(clientID, colHandle, vrep.simx_opmode_oneshot_wait)
    #print error
    #print collisionState
    error, position = vrep.simxGetObjectPosition(clientID, LSP_Handle, 600, vrep.simx_opmode_streaming)
    print error
    print position
    
    i=0
    while(1):
        if (i%2 == 0):
            vrep.simxSetJointPosition(clientID,LSP_Handle, -1 , vrep.simx_opmode_streaming)
        else:
            vrep.simxSetJointPosition(clientID,LSP_Handle, 2, vrep.simx_opmode_streaming)
        time.sleep(3)
        print "frame: " + str(i)
        error, position = vrep.simxGetObjectPosition(clientID, LSP_Handle, -1, vrep.simx_opmode_streaming)
        print error
        print position
        #error, collisionState = vrep.simxReadCollision(clientID, FLOOR_Handle, vrep.simx_opmode_oneshot_wait)
        #print "Error: " + str(error)
        #print "State: " + str(collisionState)
        i=i+1
    x =vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    #vrep.simxFinish()
else:
    print 'Failed connecting to remote API server', clientID
print 'Program ended'


