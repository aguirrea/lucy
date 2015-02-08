#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Robot Sniffer, generates poses from the robot motion
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

from simulator.SimLucy   import SimLucy
from simulator.AXAngle   import AXAngle
from Pose                import Pose
from simulator.LoadRobotConfiguration import LoadRobotConfiguration


import xml.etree.cElementTree as ET
import time, threading

class RobotSniffer:

    def __init__(self, robot):
        self.lucy = robot
        self.framesCapturedQty = 0
        self.poses = {}
        self.sniffing = False
        self.configuration = LoadRobotConfiguration()

    def startSniffing(self):
        self.sniffing = True
        frame = self.lucy.getFrame()
        self.poses[self.framesCapturedQty] = Pose(frame) # TODO take into account that angles are represented in simlulator encoding
        self.framesCapturedQty = self.framesCapturedQty + 1
        if self.sniffing:
            threading.Timer(2, self.startSniffing).start()

    def stopSniffing(self):
        self.sniffing = False

    def generateFile(self,file):
        root = ET.Element("root")
        lucy = ET.SubElement(root, "Lucy")
        for frameIt in range(self.framesCapturedQty):
            frame = ET.SubElement(lucy, "frame")
            frame.set("number" , str(frameIt))
            for joint in self.configuration.getJointsName():
                xmlJoint = ET.SubElement(frame, joint)
                joint_id = self.configuration.loadJointId(joint)
                pos = self.poses[frameIt].getValue(joint)
                xmlJointAngle = xmlJoint.set("angle" , str(pos))
        tree = ET.ElementTree(root)
        tree.write(file)