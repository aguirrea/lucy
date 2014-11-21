#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# BVH file parser and helper functions for joint calculation
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

import os
import BVHToolkit
from cgkit.bvh import Node
from BVHToolkit import Animation, Pose, Bone

class BvhImport:
    def __init__(self, file):
        bvh_serial_path = os.path.join(os.path.dirname(__file__), file)
        self.animation = Animation.from_bvh(bvh_serial_path)
        self.nodeNameIndexMapping = {}
        pose = self.animation.get_pose(1)
        for j in range(len(pose.positions)):
            self.nodeNameIndexMapping[pose.bone.node_list[j].name] = j

    def test(self):
        print self.nodeNameIndexMapping

    def listAll(self):
        for i in range(len(self.animation.frames)):
            pose = self.animation.get_pose(i)
            for j in range(len(pose.positions)):
                print str(pose.bone.node_list[j].name) + ": " + str(pose.get_position(j))

    def getNodePositions(self,nodeIndex):
        result_x={}
        result_y={}
        result_z={}
        for i in range(len(self.animation.frames)):
            pose = self.animation.get_pose(i)
            result_x[i]=pose.get_position(nodeIndex)[0]
            result_y[i]=pose.get_position(nodeIndex)[1]
            result_z[i]=pose.get_position(nodeIndex)[2]
        return result_x, result_y, result_z

    def getNodePositionsFromName(self,nodeName):
        result_x={}
        result_y={}
        result_z={}
        nodeIndex=self.nodeNameIndexMapping[nodeName]
        for i in range(len(self.animation.frames)):
            pose = self.animation.get_pose(i)
            result_x[i]=pose.get_position(nodeIndex)[0]
            result_y[i]=pose.get_position(nodeIndex)[1]
            result_z[i]=pose.get_position(nodeIndex)[2]
        return result_x, result_y, result_z

#parser = BvhImport("Example1.bvh")
#print dir(parser)
#u, v, z = parser.getNodePositionsFromName("hip")
#print v
#u, v, z = parser.getNodePositions(0)
#print v

