#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Lucy mutator operator based on pyevolve G2DListMutatorRealGaussian modified 
# using UnivariateSpline to smmoth the poses near the mutation point
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


from random import randint as rand_randint, gauss as rand_gauss, uniform as rand_uniform
from random import choice as rand_choice
import numpy as np
from scipy.interpolate import UnivariateSpline

CDefG2DListMutRealMU = 0
CDefG2DListMutRealSIGMA = 1

CDefRangeMin = 0
CDefRangeMax = 100



def interpolate(genome, which_x, which_y, offset):

   interpolationPointsQty = 10
   which_y_InterpolationNeighborhood = interpolationPointsQty/2
   minimunInterpolationNeighborhoodSize = 4

   if which_y - interpolationPointsQty/2 < 0:
      interpolationPointsQty = interpolationPointsQty - abs(which_y - which_y_InterpolationNeighborhood) * 2
      which_y_InterpolationNeighborhood = interpolationPointsQty/2

   elif which_y + interpolationPointsQty/2 > genome.getHeight()-1:
      interpolationPointsQty = interpolationPointsQty - (which_y + which_y_InterpolationNeighborhood - (genome.getHeight()-1)) * 2
      which_y_InterpolationNeighborhood = interpolationPointsQty/2

   if minimunInterpolationNeighborhoodSize > 4:
      x = np.ndarray(interpolationPointsQty)
      y = np.ndarray(interpolationPointsQty)
      
      for k in xrange(interpolationPointsQty):
         poseToSmooth = which_y - which_y_InterpolationNeighborhood + k
         x[k] = poseToSmooth
         y[k] = genome[poseToSmooth][which_x]
      
      spl = UnivariateSpline(x, y)
      spl.set_smoothing_factor(0.5)

      for k in xrange(interpolationPointsQty):
         #print "before ", genome[int(x[k])][which_x], "now ", spl(int(x[k])), "k ", k, "diff ", genome[int(x[k])][which_x]-spl(int(x[k])), "offset ", offset
         genome.setItem(int(x[k]), which_x, spl(int(x[k])))   


def G2DListMutatorRealGaussianSpline(genome, **args):
   """ A gaussian mutator for G2DList of Real with UnivariateSpline for smoothing 

   Accepts the *rangemin* and *rangemax* genome parameters, both optional. Also
   accepts the parameter *gauss_mu* and the *gauss_sigma* which respectively
   represents the mean and the std. dev. of the random distribution.

   """
   if args["pmut"] <= 0.0: return 0
   height, width = genome.getSize()
   elements = height * width

   mutations = args["pmut"] * elements

   mu = genome.getParam("gauss_mu")
   sigma = genome.getParam("gauss_sigma")
   
   if mu is None:
      mu = CDefG2DListMutRealMU
   
   if sigma is None:
      sigma = CDefG2DListMutRealSIGMA

   if mutations < 1.0:
      mutations = 0
      
      for i in xrange(genome.getHeight()):
         for j in xrange(genome.getWidth()):
            if Util.randomFlipCoin(args["pmut"]):
               offset = rand_gauss(mu, sigma)
               final_value = genome[i][j] + offset

               final_value = min(final_value, genome.getParam("rangemax", CDefRangeMax))
               final_value = max(final_value, genome.getParam("rangemin", CDefRangeMin))

               genome.setItem(i, j, final_value, offset)
               interpolate(genome, j, i)

               mutations += 1
   else: 

      for it in xrange(int(round(mutations))):
         which_x = rand_randint(0, genome.getWidth()-1)  #joint to mutate
         which_y = rand_randint(0, genome.getHeight()-1) #pose to mutate
         offset = rand_gauss(mu, sigma)
         final_value = genome[which_y][which_x] + offset

         #to be sure that the value is less than the rangemax and more than the rangemin value
         final_value = min(final_value, genome.getParam("rangemax", CDefRangeMax))
         final_value = max(final_value, genome.getParam("rangemin", CDefRangeMin))

         #valueBeforeMutation = genome[which_y][which_x]
         genome.setItem(which_y, which_x, final_value)
         interpolate(genome, which_x, which_y, offset)


   return int(mutations)
