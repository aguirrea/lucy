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
from pyevolve import DBAdapters
from pyevolve import Statistics

from datatypes.DTIndividualProperty             import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero, DTIndividualPropertyVanillaEvolutive
from datatypes.DTIndividualGeneticMaterial      import DTIndividualGeneticMaterial, DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix
from Individual                                 import Individual
from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
#from simulator.LoadRobotConfiguration import LoadRobotConfiguration

import time
import os
import glob

initialPopulationSetted = False
gaEngine = None

def setInitialPopulation (ga_engine):

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
    
    population = ga_engine.getPopulation()
    popSize = len(population)

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

def createOwnGen(ga_engine):
    # persist best individual at the moment
    conf = LoadSystemConfiguration() #TODO make an object to encapsulate this kind of information
    geneticPoolDir = os.getcwd()+conf.getDirectory("Genetic Pool")
    gen = ga_engine.getCurrentGeneration()
    best = ga_engine.bestIndividual()
    score = best.getRawScore()
    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = str(score) + "-" + timestr + "-" + str(gen) + ".xml"
    prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(best)))
    bestIndividual.persist(geneticPoolDir + filename)

    return False

def chromosomeToLucyGeneticMatrix(chromosome):
    geneticMatrix = [[chromosome[i][j] for j in xrange(chromosome.getWidth())] for i in xrange(chromosome.getHeight())] #debería pedir solo los joints implementados
    return geneticMatrix

# This function is the evaluation function
def eval_func(chromosome):
    if not initialPopulationSetted:
        setInitialPopulation(gaEngine)    
    #prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    prop = DTIndividualPropertyVanillaEvolutive()
    individual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(chromosome)))
    fitness = individual.execute() #return the fitness resulting from the simulator execution
    return fitness

# This function is the termination criteria for the algorithm
def ConvergenceCriteria(ga_engine):
   pop = ga_engine.getPopulation()
   return pop[0] == pop[len(pop)-1]

def run_main():
    initialPopulationSize = 50
    generations = 70   
    conf = LoadSystemConfiguration() #TODO make an object to encapsulate this kind of information
    # Genome instance
    genome = G2DList.G2DList(164, 18)
    genome.setParams(rangemin=0, rangemax=360)

    # The evaluator function (objective function)
    genome.evaluator.set(eval_func)
    genome.crossover.set(Crossovers.G2DListCrossoverSingleHPoint)
    #genome.mutator.set(Mutators.G2DListMutatorIntegerRange)
    genome.mutator.set(Mutators.G2DListMutatorIntegerGaussian)

    # Genetic Algorithm Instance
    ga = GSimpleGA.GSimpleGA(genome)
    ga.setGenerations(generations)    #TODO class atribute
    ga.setPopulationSize(initialPopulationSize) #TODO class atribute
    ga.setMutationRate(0.2)
    #ga.selector.set(Selectors.GRouletteWheel) 
    #ga.terminationCriteria.set(ConvergenceCriteria)

    # Create DB Adapter and set as adapter
    sqlite_adapter = DBAdapters.DBSQLite(identify="Lucy walk", resetDB=True)
    ga.setDBAdapter(sqlite_adapter)
                        
    #callback to persist best individual of each generation
    ga.stepCallback.set(createOwnGen)

    #keep a reference to the genetic algorithm engine
    global gaEngine
    gaEngine = ga

    # Do the evolution, with stats dump
    # frequency of 10 generations
    ga.evolve(freq_stats=2)

    # Best individual
    best = ga.bestIndividual()
    score = best.getRawScore()
    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = str(score) + "-" + timestr + "-" + str(generations) + ".xml"
    prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(best)))
    geneticPoolDir = os.getcwd()+conf.getDirectory("Genetic Pool")
    bestIndividual.persist(geneticPoolDir + filename)

if __name__ == "__main__":
   run_main()