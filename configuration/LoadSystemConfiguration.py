#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Parser for system configuration file
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
confFile = os.getcwd()+"/configuration/SystemConf.xml"

class LoadSystemConfiguration:
    
    def __init__(self):
        self.directoryValueMapping = {}
        self.fileValueMapping = {}
        self.propertyValueMapping = {}
        xmldoc = minidom.parse(confFile)
        itemlist = xmldoc.getElementsByTagName("Directory") 
        for i in itemlist:
            for j in xrange(len(i.getElementsByTagName("Name"))):
                name = i.getElementsByTagName("Name")[j]
                id   = i.getElementsByTagName("Value")[j]
                #print "Name:" + name.childNodes[0].toxml() + " Id: " + id.childNodes[0].toxml()
                self.directoryValueMapping[(name.childNodes[0].toxml())] = id.childNodes[0].toxml() 
        itemlist = xmldoc.getElementsByTagName("File") 
        for i in itemlist:
            for j in xrange(len(i.getElementsByTagName("Name"))):
                name = i.getElementsByTagName("Name")[j]
                id   = i.getElementsByTagName("Value")[j]
                #print "Name:" + name.childNodes[0].toxml() + " Id: " + id.childNodes[0].toxml()
                self.fileValueMapping[(name.childNodes[0].toxml())] = id.childNodes[0].toxml() 
        
        itemlist = xmldoc.getElementsByTagName("Property") 
        for i in itemlist:
            for j in xrange(len(i.getElementsByTagName("Name"))):
                name = i.getElementsByTagName("Name")[j]
                id   = i.getElementsByTagName("Value")[j]
                #print "Name:" + name.childNodes[0].toxml() + " Id: " + id.childNodes[0].toxml()
                self.propertyValueMapping[(name.childNodes[0].toxml())] = id.childNodes[0].toxml() 
            
    def getDirectory(self, name):
        return(self.directoryValueMapping[name])

    def getFile(self, name):
        return(self.fileValueMapping[name])

    def getProperty(self, name):
        return(self.propertyValueMapping[name])

    def getVrepNotImplementedBioloidJoints(self):
        return(self.propertyValueMapping["Vrep not implemented joints"].split())
    
    
#conf = LoadSystemConfiguration()
#print conf.getDirectory("Transformed mocap Files")
