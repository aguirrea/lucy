#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# SimpleTestBioloidSet example using the Simulator API
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

from ..simulator.Simulator import Simulator
from ..simulator.AXAngle import AXAngle
from ..simulator.LoadPoses import LoadPoses

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
        for index in xrange(frameQty):
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





