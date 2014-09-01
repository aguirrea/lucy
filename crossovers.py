#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Lucy crossover function
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

import pyevolve.Util as Util
from random import randint
#todo implement the distance function
def LucyXSinglePoint(genome, **args):
   sister = None
   brother = None
   gMom = args["mom"]
   gDad = args["dad"]

   mom_poses = gMom.num_poses()

   cut = randint(0, mom_poses-1)

   if args["count"] >= 1:
      sister = gMom.clone()
      sister.resetStats()
      sister.poses[cut:] = gDad.poses[cut:]

   if args["count"] == 2:
      brother = gDad.clone()
      brother.resetStats()
      brother.polygons[cut:] = gMom.poses[cut:]

   return (sister, brother)

