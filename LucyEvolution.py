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

from datatypes.DTIndividualProperty             import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero, DTIndividualPropertyVanillaEvolutive
from datatypes.DTIndividualGeneticMaterial      import DTIndividualGeneticMaterial, DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix
from Individual                                 import Individual
from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
#from simulator.LoadRobotConfiguration import LoadRobotConfiguration

import time
import os
import glob

initialPopulationSetted = False
def createOwnGen(ga_engine):
    gen = ga_engine.getCurrentGeneration()

    prop = DTIndividualPropertyCMUDaz()    
    propVanilla = DTIndividualPropertyVanilla()
    balieroProp = DTIndividualPropertyBaliero()

    conf = LoadSystemConfiguration()

    CMUxmlDir = os.getcwd()+conf.getDirectory("Transformed CMU mocap Files")
    GAwalkDir = os.getcwd()+conf.getDirectory("GAWalk Files")
    UIBLHDir = os.getcwd()+conf.getDirectory("UIBLH mocap Files")
    BalieroDir = os.getcwd()+conf.getDirectory("Baliero transformed walk Files")
    ADHOCDir = os.getcwd()+conf.getDirectory("ADHOC Files")
    geneticPoolDir = os.getcwd()+conf.getDirectory("Genetic Pool")
    if gen == 0:
        population = ga_engine.getPopulation()
        popSize = len(population)
        print popSize
        individualCounter = 0
        for filename in glob.glob(os.path.join(CMUxmlDir, '*.xml')):
            print individualCounter, " individuals processed!"
            print 'inserting individual: ' + filename + " into the initial population"
            walk = Individual(prop, DTIndividualGeneticTimeSerieFile(filename))
            geneticMatrix = walk.getGenomeMatrix()
            if individualCounter < popSize:
                adan = population[individualCounter]
                for i in xrange(adan.getHeight()):
                    if i == len(geneticMatrix):
                            break
                    for j in xrange(adan.getWidth()):
                        adan.setItem(i,j,geneticMatrix[i][j])
                population[individualCounter]=adan
                individualCounter = individualCounter + 1
            else:
                break
        global initialPopulationSetted
        initialPopulationSetted = True
    else:
        # persist moment best individual
        timestr = time.strftime("%Y%m%d-%H%M%S")
        filename = timestr + "-" + str(gen) + ".xml"
        prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
        bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(ga_engine.bestIndividual())))
        bestIndividual.persist(geneticPoolDir + filename)

    return False

def chromosomeToLucyGeneticMatrix(chromosome):
    geneticMatrix = [[chromosome[i][j] for j in xrange(chromosome.getWidth())] for i in xrange(chromosome.getHeight())] #debería pedir solo los joints implementados
    return geneticMatrix

# This function is the evaluation function
def eval_func(chromosome):
    fitness = 0
    if initialPopulationSetted == True:
        print "***********************---------------------------------------------------"
        #prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
        prop = DTIndividualPropertyVanillaEvolutive()
        individual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(chromosome)))
        fitness = individual.execute() #return the fitness resulting from the simulator execution
    return fitness

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
    ga.setGenerations(30)
    ga.setPopulationSize(51)

    #the first call sets the initial population
    ga.stepCallback.set(createOwnGen)

    # Do the evolution, with stats dump
    # frequency of 10 generations
    ga.evolve(freq_stats=2)

    # Best individual
    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = timestr + ".xml"
    prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(ga.bestIndividual())))
    geneticPoolDir = os.getcwd()+conf.getDirectory("Genetic Pool")
    bestIndividual.persist(geneticPoolDir + filename)

if __name__ == "__main__":
   run_main()