#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Pose generator from Lucy real robot
# with graphic representation in vrep
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
import time
import threading

import sys

import signal

from simulator.Actuator import Actuator
from simulator.Communication import CommSerial
from simulator.LoadRobotConfiguration import LoadRobotConfiguration
from simulator.Lucy import SimulatedLucy
from simulator.Pose import Pose


class PersistPhysicalTask:
    def __init__(self):
        comm_tty = CommSerial()
        comm_tty.connect()
        self.actuator_tty = Actuator(comm_tty)
        self.config = LoadRobotConfiguration()
        self.lucySimulatedRobot = SimulatedLucy(True)
        self.threadRunning = False

        self.root = ET.Element("root")
        self.lucy = ET.SubElement(self.root, "Lucy")
        self.pose = Pose()
        self.frameIt = 0

        signal.signal(signal.SIGINT, self.signalHandler)
        self.thread = threading.Thread(target=self.captureTimmer, name='Daemon')

        #signal.pause()

    def startCapturingMotion(self):
        self.threadRunning = True
        self.thread.setDaemon(True)
        self.thread.start()
        pass

    def stopCapturingMotion(self):
        self.threadRunning = False
        tree = ET.ElementTree(self.root)
        tree.write("poses.xml")
        self.thread.do_run = False
        self.thread.join()

    def signalHandler(self):
        print ('You pressed Ctrl+C!')
        self.stopCapturingMotion()
        sys.exit(0)

    def captureTimmer(self):
        while(self.threadRunning):
            frame = ET.SubElement(self.lucy, "frame")
            frame.set("number", str(self.frameIt))
            for joint in self.config.getJointsName():
                xmlJoint = ET.SubElement(frame, joint)
                joint_id = self.config.loadJointId(joint)
                pos = self.actuator_tty.get_position(joint_id).toDegrees()
                #print joint, ":", joint_id, ": ", pos
                if joint == "R_Knee" or joint == "R_Ankle_Pitch" or joint == "L_Hip_Pitch" or joint == "L_Ankle_Roll" or joint == "R_Ankle_Roll" or joint == "L_Hip_Roll" or joint == "R_Hip_Roll" or joint == "R_Shoulder_Pitch":
                    self.pose.setValue(joint, 300 - pos)
                else:
                    self.pose.setValue(joint, pos)
                xmlJointAngle = xmlJoint.set("angle", str(pos))
            self.lucySimulatedRobot.executePose(self.pose)
            self.frameIt += 1
            time.sleep(0.2)
            print "i'm alive"

print "iniciate"
ppt = PersistPhysicalTask()
print "thread creado"
ppt.startCapturingMotion()
print "thread iniciado"
ri = raw_input('press any key to finalize the motion capture\n')
ppt.stopCapturingMotion()
