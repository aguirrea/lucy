#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Pose generator from Lucy real robot
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

import xml.etree.cElementTree as ET
from LoadRobotConfiguration import LoadRobotConfiguration
import Communication
from Communication import CommSerial
import Actuator

comm_tty = CommSerial()  #TODO read a configuration file to use the correct parameters for CommSimulator
comm_tty.connect()
actuator_tty = Actuator.Actuator(comm_tty)

root = ET.Element("root")
lucy = ET.SubElement(root, "Lucy")

config = LoadRobotConfiguration()
frameIt = 0
ri = raw_input('presione \'c\' para capturar otra tecla para terminar \n')
while(ri == 'c'):
    frame = ET.SubElement(lucy, "frame")
    frame.set("number" , str(frameIt))
    for joint in config.getJointsName():
        xmlJoint = ET.SubElement(frame, joint)
        joint_id = config.loadJointId(joint)
        print joint_id
        pos = actuator_tty.get_position(joint_id)
        #actuator_tty.move_actuator(frameIt,500,700)
        #print pos
        #xmlJointAngle = xmlJoint.set("angle" , "3")
        xmlJointAngle = xmlJoint.set("angle" , str(pos))
    ri = raw_input('presione \'c\' para capturar otra tecla para terminar \n')
    frameIt = frameIt + 1

tree = ET.ElementTree(root)
tree.write("poses.xml")
