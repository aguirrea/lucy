from Simulator import Simulator
from AXAngle import AXAngle
from LoadPoses import LoadPoses

import math
import os

standadRemoteApiPort=19997
localhost='127.0.0.1'
genetic_bioloid=os.getcwd()+"/models/genetic_bioloid.ttt"

print 'Program started'
angle = AXAngle()
lp = LoadPoses()
sim = Simulator()

clientID = sim.connectVREP()

if clientID !=-1:
    print 'Connected to remote API server'
    sim.loadscn(clientID, genetic_bioloid)
    sim.startSim(clientID,True)
    frameQty=lp.getFrameQty()
    pos1x, pos1y=sim.getBioloidPlannarPosition(clientID)
    end=False
    while not end:
        for index in range(frameQty):
            pose=lp.getFramePose(index)
            #sim.pauseSim(clientID)
            for joint in pose.keys():
                angle.setValue(pose[joint])
                sim.setJointPosition(clientID,joint,angle.toVrep())
            #sim.resumePauseSim(clientID)
            sim.printJointPositions(clientID)
            print index
        end=not sim.isRobotUp(clientID)
        print "la condicion de fin es: ", end
    pos2x, pos2y=sim.getBioloidPlannarPosition(clientID) 
    print math.sqrt((pos2x-pos1x)**2 + (pos2y-pos1y)**2)   
    error=sim.finishSimulation(clientID)
else:
    print 'Failed connecting to remote API server', clientID
print 'Program ended'





