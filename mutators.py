#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Lucy mutator operator based on pyevolve G2DListMutatorRealGaussian
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
#import matplotlib.pyplot as plt

CDefG2DListMutRealMU = 0
CDefG2DListMutRealSIGMA = 1

CDefRangeMin = 0
CDefRangeMax = 100

interpolationPointsQty = 10
num = interpolationPointsQty/2


def interpolate(genome, which_x, which_y):

   if which_y - interpolationPointsQty/2 < 0:
      interpolationPointsQty = interpolationPointsQty - abs(which_y - interpolationPointsQty/2) * 2
      num = interpolationPointsQty/2

   elif which_y + interpolationPointsQty/2 > genome.getHeight()-1:
      interpolationPointsQty = interpolationPointsQty - (which_y + interpolationPointsQty/2 - (genome.getHeight()-1)) * 2
      num = interpolationPointsQty/2

   if interpolationPointsQty > 4:
      x = np.ndarray(interpolationPointsQty)
      y = np.ndarray(interpolationPointsQty)
      for k in xrange(interpolationPointsQty):
         poseToSmooth = which_y - num + k
         x[k] = poseToSmooth
         y[k] = genome[poseToSmooth][which_x]
      spl = UnivariateSpline(x, y)
      spl.set_smoothing_factor(0.3) #2 - 0,3
      #plt.plot(x, y, 'ro', ms=5)
      #plt.plot(x, spl(x), 'g', lw=3)

      for k in xrange(interpolationPointsQty):
         #print "xk", x[k], "intxk ", int(x[k])
         #print "antes ", genome[int(x[k])][which_x], "ahora ", spl(int(x[k])), "k ", k, "diff ", genome[int(x[k])][which_x]-spl(int(x[k])), "offset ", offset
         genome.setItem(int(x[k]), which_x, spl(int(x[k])))   



def G2DListMutatorRealGaussianSpline(genome, **args):
   """ A gaussian mutator for G2DList of Real 

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
               final_value = genome[i][j] + rand_gauss(mu, sigma)

               final_value = min(final_value, genome.getParam("rangemax", Consts.CDefRangeMax))
               final_value = max(final_value, genome.getParam("rangemin", Consts.CDefRangeMin))

               genome.setItem(i, j, final_value)
               

               mutations += 1
   else: 

      for it in xrange(int(round(mutations))):
         which_x = rand_randint(0, genome.getWidth()-1)  #joint to mutate
         which_y = rand_randint(0, genome.getHeight()-1) #pose to mutate
         #print "which_x ", which_x
         #print "which_y ", which_y
         offset = rand_gauss(mu, sigma)
         final_value = genome[which_y][which_x] + offset

         final_value = min(final_value, genome.getParam("rangemax", CDefRangeMax))
         final_value = max(final_value, genome.getParam("rangemin", CDefRangeMin))

         #valueBeforeMutation = genome[which_y][which_x]
         genome.setItem(which_y, which_x, final_value)

         if which_y - interpolationPointsQty/2 < 0:
            interpolationPointsQty = interpolationPointsQty - abs(which_y - interpolationPointsQty/2) * 2
            num = interpolationPointsQty/2

         elif which_y + interpolationPointsQty/2 > genome.getHeight()-1:
            interpolationPointsQty = interpolationPointsQty - (which_y + interpolationPointsQty/2 - (genome.getHeight()-1)) * 2
            num = interpolationPointsQty/2

         if interpolationPointsQty > 4:
            x = np.ndarray(interpolationPointsQty)
            y = np.ndarray(interpolationPointsQty)
            for k in xrange(interpolationPointsQty):
               poseToSmooth = which_y - num + k
               x[k] = poseToSmooth
               y[k] = genome[poseToSmooth][which_x]
            spl = UnivariateSpline(x, y)
            spl.set_smoothing_factor(0.3) #2 - 0,3
            #plt.plot(x, y, 'ro', ms=5)
            #plt.plot(x, spl(x), 'g', lw=3)

            for k in xrange(interpolationPointsQty):
               #print "xk", x[k], "intxk ", int(x[k])
               #print "antes ", genome[int(x[k])][which_x], "ahora ", spl(int(x[k])), "k ", k, "diff ", genome[int(x[k])][which_x]-spl(int(x[k])), "offset ", offset
               genome.setItem(int(x[k]), which_x, spl(int(x[k])))   

   return int(mutations)
