#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# Andrés Vasilev
# MINA/INCO/UDELAR
#
# Sniffer for the serial port
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
import binascii
from simulator.Actuator import Actuator
from simulator.Communication import CommSerial
from simulator.LoadRobotConfiguration import LoadRobotConfiguration
from simulator.Lucy import SimulatedLucy
from simulator.Pose import Pose

class SerialSniffer:
    def __init__(self):
        self.comm_tty = CommSerial()
        self.comm_tty.connect()

    def getHexByte(self):
        return  binascii.hexlify(self.comm_tty.getByte())


sniffer = SerialSniffer()
while True:
    byte = sniffer.getHexByte()
    print byte
