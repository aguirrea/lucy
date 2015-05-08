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

from DTIndividualProperty import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero
from Individual import Individual

import time

# This function is the evaluation function, we want
# to give high score to more zero'ed chromosomes
def eval_func(chromosome):
    prop = DTIndividualPropertyCMUDaz()
    individual = Individual(prop, chromosome)
    return individual.execute()

def run_main():
    # Genome instance
    genome = G2DList.G2DList(164, 14)
    genome.setParams(rangemin=0, rangemax=360)

    # The evaluator function (objective function)
    genome.evaluator.set(eval_func)
    genome.crossover.set(Crossovers.G2DListCrossoverSingleHPoint)
    genome.mutator.set(Mutators.G2DListMutatorIntegerRange)

    # Genetic Algorithm Instance
    ga = GSimpleGA.GSimpleGA(genome)
    ga.setGenerations(800)

    # Do the evolution, with stats dump
    # frequency of 10 generations
    ga.evolve(freq_stats=100)

    # Best individual
    #print ga.bestIndividual()

    #crear individuo con material genético ga.bestIndividual() y persistirlo
    timestr = time.strftime("%Y%m%d-%H%M%S")
    print timestr


if __name__ == "__main__":
   run_main()