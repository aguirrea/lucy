#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Test for the robot sniffer
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
from parser.LoadPoses    import LoadPoses
from RobotSniffer        import RobotSniffer

import math
import os
import time

print 'Program started' 
angle = AXAngle()

lucy = SimLucy(True)

sniffer = RobotSniffer(lucy)
sniffer.startSniffing()

while  lucy.isLucyUp():
    time.sleep(1)
    print 'sniffing'
    
sniffer.stopSniffing()
lucy.stopLucy()
sniffer.generateFile("sniffed_walk1.xml")

print 'Program ended'




