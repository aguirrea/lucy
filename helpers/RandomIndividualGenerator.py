#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Generates a individual of random length with random joint values
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

import random
import xml.etree.cElementTree as ET

from tests.simulator.LoadRobotConfiguration import LoadRobotConfiguration
from tests.simulator.Pose import Pose


class RandomIndividualGenerator:
    def __init__(self, file):
        self.framesQty = random.randint(30, 50)
        self.frames = {}
        self.configuration = LoadRobotConfiguration()
        self.joints = self.configuration.getJointsName()
        self.file = file

        for frameIt in range(self.framesQty):
            pose = {}
            for joint in self.joints:
                pose[joint] = random.uniform(0.0, 300)

            self.frames[frameIt] = Pose(pose)

    def generateFile(self):
        root = ET.Element("root")
        lucy = ET.SubElement(root, "Lucy")
        for frameIt in range(self.framesQty):
            frame = ET.SubElement(lucy, "frame")
            frame.set("number", str(frameIt))
            for joint in self.configuration.getJointsName():
                xmlJoint = ET.SubElement(frame, joint)
                pos = self.frames[frameIt].getValue(joint)
                xmlJoint.set("angle", str(pos))
        tree = ET.ElementTree(root)
        tree.write(self.file)

'''randInd = RandomIndividualGenerator("borrame.xml")
randInd.generateFile()'''