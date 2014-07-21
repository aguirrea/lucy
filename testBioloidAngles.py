from Simulator import Simulator
from AXAngle   import AXAngle
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
    angle.setDegreeValue(150)    
    sim.setJointPosition(clientID,"R_Hip_Pitch",angle.toVrep())
    sim.setJointPosition(clientID,"L_Hip_Pitch",angle.toVrep())    
    frameQty=lp.getFrameQty()
    pos1x, pos1y=sim.getBioloidPlannarPosition(clientID)
    end=False
    while not end:
        for i in range(30):
            angle.setDegreeValue(150-i)
            print "voy a setear ax12 value en cadera izquierda: ", 150 - i
            sim.setJointPosition(clientID,"L_Hip_Pitch",angle.toVrep())
            angle.setDegreeValue(i*2)
            print "voy a setear ax12 value en rodilla derecha: ", i
            sim.setJointPosition(clientID,"R_Knee",angle.toVrep())
            
        for i in range(30):
            angle.setDegreeValue(150-i)
            sim.setJointPosition(clientID,"R_Hip_Pitch",angle.toVrep()) 
            sim.setJointPosition(clientID,"L_Knee",angle.toVrep()) 
            
        for i in range(30):
            angle.setDegreeValue(120+i)
            sim.setJointPosition(clientID,"R_Hip_Pitch",angle.toVrep())
            sim.setJointPosition(clientID,"L_Hip_Pitch",angle.toVrep())
            
        end=not sim.isRobotUp(clientID)
        print "la condicion de fin es: ", end
    pos2x, pos2y=sim.getBioloidPlannarPosition(clientID) 
    print math.sqrt((pos2x-pos1x)**2 + (pos2y-pos1y)**2)   
    error=sim.finishSimulation(clientID)
    
else:
    print 'Failed connecting to remote API server', clientID
print 'Program ended'





