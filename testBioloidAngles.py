#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Test case for the Simulator class
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

from Simulator import Simulator
from AXAngle   import AXAngle

import math
import os

standadRemoteApiPort=19997
localhost='127.0.0.1'
genetic_bioloid=os.getcwd()+"/models/genetic_bioloid.ttt"

print 'Program started' 
angle = AXAngle()
sim = Simulator()

clientID = sim.connectVREP()

if clientID !=-1:
    print 'Connected to remote API server'
    sim.loadscn(clientID, genetic_bioloid)
    sim.startSim(clientID,True)
    angle.setDegreeValue(150)    
    sim.setJointPosition(clientID,"R_Hip_Pitch",angle.toVrep())
    sim.setJointPosition(clientID,"L_Hip_Pitch",angle.toVrep())    
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





