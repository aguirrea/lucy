#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Lucy evolution
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


from pyevolve import G2DList
from pyevolve import GSimpleGA
from pyevolve import Selectors
from pyevolve import Crossovers
from pyevolve import Mutators

from DTIndividualProperty             import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero
from DTIndividualGeneticMaterial      import DTIndividualGeneticMaterial, DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix
from Individual                       import Individual

import time

def chromosomeToLucyGeneticMatrix(chromosome):
    geneticMatrix = [[chromosome[i][j] for j in xrange(chromosome.getWidth())] for i in xrange(chromosome.getHeight())] #debería pedir solo los joints implementados
    return geneticMatrix

# This function is the evaluation function
def eval_func(chromosome):
    prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    individual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(chromosome)))
    return individual.execute() #return the fitness resulting from the simulator execution

def run_main():
    # Genome instance
    genome = G2DList.G2DList(164, 18)
    genome.setParams(rangemin=0, rangemax=360)

    # The evaluator function (objective function)
    genome.evaluator.set(eval_func)
    genome.crossover.set(Crossovers.G2DListCrossoverSingleHPoint)
    genome.mutator.set(Mutators.G2DListMutatorIntegerRange)

    # Genetic Algorithm Instance
    ga = GSimpleGA.GSimpleGA(genome)
    ga.setGenerations(20)

    # Do the evolution, with stats dump
    # frequency of 10 generations
    ga.evolve(freq_stats=2)

    # Best individual
    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = timestr + ".xml"
    prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(ga.bestIndividual())))
    bestIndividual.persist(filename)

if __name__ == "__main__":
   run_main()