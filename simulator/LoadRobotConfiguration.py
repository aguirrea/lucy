#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Parser for robot configuration file
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

from xml.dom import minidom
import os
confFile = os.getcwd()+"/configuration/RobotConf.xml"

#TODO move this to configuration folder?
class LoadRobotConfiguration:
    
    def __init__(self):
        self.jointAngleMapping = {}
        xmldoc = minidom.parse(confFile)
        itemlist = xmldoc.getElementsByTagName("Joint") 
        for i in itemlist:
            name = i.getElementsByTagName("Name")[0]
            id   = i.getElementsByTagName("Id")[0]
            #print "Name:" + name.childNodes[0].toxml() + " Id: " + id.childNodes[0].toxml()
            self.jointAngleMapping[(name.childNodes[0].toxml())] = int(id.childNodes[0].toxml())        
            
    def loadJointId(self, jointName):
        return(self.jointAngleMapping[jointName])
    
    def getJointsName(self):
        return sorted(self.jointAngleMapping.keys())

    
#conf = LoadRobotConfiguration()
#for joint in conf.getJointsName():
#    print conf.loadJointId(joint)
