#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Program to capture the natural value of each joint
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


from simulator.SimLucy    import SimLucy
from Pose                 import Pose
from simulator.AXAngle    import AXAngle
from errors.VrepException import VrepException
from simulator.LoadRobotConfiguration import LoadRobotConfiguration
import time 
import xml.etree.cElementTree as ET


print "program started"
robotConfig = LoadRobotConfiguration()
lucy = SimLucy(True)
error = False
try:
    error, reposeFrame = lucy.getFrame()
    while (error):
        error, reposeFrame = lucy.getFrame()
        time.sleep(0.5)
    newPose = Pose(reposeFrame)
except VrepException, e:
    print str(e)

root = ET.Element("root")
lucyPersistence = ET.SubElement(root, "Lucy")
configuration = LoadRobotConfiguration()


for i in xrange(80):
    print "capture: ", i
    frame = ET.SubElement(lucyPersistence, "frame")
    frame.set("number" , str(i))
    for key in robotConfig.getJointsName() :
        xmlJoint = ET.SubElement(frame, key)
        joint_id = configuration.loadJointId(key)
        degreeAngle = 150-(reposeFrame[key]*float(60))
        xmlJointAngle = xmlJoint.set("angle" , str(degreeAngle))
    lucy.executePose(newPose)
lucy.stopLucy()
tree = ET.ElementTree(root)
tree.write("repose.xml")






