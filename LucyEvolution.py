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
import crossovers

initialPopulationSetted = False
gaEngine = None
NUMBER_GENERATIONS_CONVERGENCE_CRITERIA = 20
max_score = 0
max_score_generation = 0
convergenceCriteria = False



def getPopulationAverage(population):
    average = 0
    for p in population:
        score = p.getRawScore()
        average = average + score
    return average/len(population)


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
    '''
    if individualCounter < popSize:
        for filename in glob.glob(os.path.join(GAwalkDir, '*.xml')):
            print individualCounter, " individuals processed!"
            print 'inserting individual: ' + filename + " into the initial population"
            walk = Individual(propVanilla, DTIndividualGeneticTimeSerieFile(filename))
            geneticMatrix = walk.getGenomeMatrix()
            if individualCounter < popSize-1:
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
    '''    
    if individualCounter < popSize:
        for filename in glob.glob(os.path.join(CMUxmlDir, '*.xml')):
            print individualCounter, " individuals processed!"
            print 'inserting individual: ' + filename + " into the initial population"
            walk = Individual(prop, DTIndividualGeneticTimeSerieFile(filename))
            geneticMatrix = walk.getGenomeMatrix()
            if individualCounter < popSize-1:
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

def chromosomeToLucyGeneticMatrix(chromosome):
    geneticMatrix = [[chromosome[i][j] for j in xrange(chromosome.getWidth())] for i in xrange(chromosome.getHeight())] #debería pedir solo los joints implementados
    return geneticMatrix

def generationCallback(ga_engine):
    # persist best individual at the moment
    conf = LoadSystemConfiguration() #TODO make an object to encapsulate this kind of information
    geneticPoolDir = os.getcwd()+conf.getDirectory("Genetic Pool")
    gen = ga_engine.getCurrentGeneration()
    best = ga_engine.bestIndividual()
    score = best.getRawScore()

    #check if the convergence criteria has been reached
    global max_score, max_score_generation, convergenceCriteria
    if score > max_score:
        max_score = score
        max_score_generation = gen
    else:
        if gen - max_score > NUMBER_GENERATIONS_CONVERGENCE_CRITERIA:
            convergenceCriteria = True

    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = str(score) + "-" + timestr + "-" + str(gen) + ".xml"
    prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(best)))
    bestIndividual.persist(geneticPoolDir + filename)
    ga_engine.getDBAdapter().commit()

    ##population = ga_engine.getPopulation()
    ##averagePopulation = getPopulationAverage(population)
    #averageGeneration = getPopulationAverage(gen)
    ##print "current population raw score average: ", averagePopulation
    #print "current generation raw score average: ", averageGeneration
    return False

# This function is the evaluation function
def eval_func(chromosome):
    FITNESS_AVERAGE_ITERATIONS = 3
    if not initialPopulationSetted:
        setInitialPopulation(gaEngine)    
    #prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    prop = DTIndividualPropertyVanillaEvolutive()
    individual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(chromosome)))
    fitness = individual.execute() #return the fitness resulting from the simulator execution
    #if we reach the fittest a average of FITNESS_AVERAGE_ITERATIONS executions is calculated as fitness to avoid the stocastic situation
    if fitness > max_score:
        for i in range(FITNESS_AVERAGE_ITERATIONS-1):
            fitness = fitness + individual.execute()
        fitness = fitness/FITNESS_AVERAGE_ITERATIONS
    return fitness

# This function is the termination criteria for the algorithm
def ConvergenceCriteria(ga_engine):
    global convergenceCriteria
    return convergenceCriteria

def run_main():
    initialPopulationSize = 51
    generations = 90
    conf = LoadSystemConfiguration() #TODO make an object to encapsulate this kind of information
    # Genome instance
    framesQty = int(conf.getProperty("Individual frames quantity"))
    genome = G2DList.G2DList(framesQty, 18)
    genome.setParams(rangemin=0, rangemax=360)

    # The evaluator function (objective function)
    genome.evaluator.set(eval_func)
    genome.crossover.set(crossovers.G2DListCrossoverSingleNearHPoint)
    #genome.crossover.set(crossovers.G2DListCrossoverSingleNearHPointImprove)
    # Genetic Algorithm Instance
    ga = GSimpleGA.GSimpleGA(genome)
    ga.setGenerations(generations)    #TODO class atribute
    ga.setPopulationSize(initialPopulationSize) #TODO class atribute

    #genome.mutator.set(Mutators.G2DListMutatorIntegerRange)
    genome.mutator.set(Mutators.G2DListMutatorRealGaussian)
    #genome.mutator.set(Mutators.G2DListMutatorRealGaussianGradient)
    ga.setMutationRate(0.1)
    
    #ga.selector.set(Selectors.GRankSelector)
    ga.selector.set(Selectors.GTournamentSelector)
    '''For crossover probability, maybe it is the ratio of next generation population born by crossover operation. 
    While the rest of population...maybe by previous selection or you can define it as best fit survivors'''
    ga.setCrossoverRate(0.9) 
    
    #ga.selector.set(Selectors.GTournamentSelector)
    #ga.selector.set(Selectors.GRouletteWheel)
    ga.setElitism(True)
    '''Set the number of best individuals to copy to the next generation on the elitism'''
    ga.setElitismReplacement(initialPopulationSize/2)
    ga.terminationCriteria.set(ConvergenceCriteria)

    # Create DB Adapter and set as adapter
    sqlite_adapter = DBAdapters.DBSQLite(identify="Lucy walk", resetDB=True)
    ga.setDBAdapter(sqlite_adapter)
                        
    #callback to persist best individual of each generation
    ga.stepCallback.set(generationCallback)

    #keep a reference to the genetic algorithm engine
    global gaEngine
    gaEngine = ga

    # Do the evolution, with stats dump
    # frequency of 2 generations
    ga.evolve(freq_stats=1)

    # Best individual
    best = ga.bestIndividual()
    score = best.getRawScore()
    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = str(score) + "-" + timestr + "-" + str(generations) + ".xml"
    prop = DTIndividualPropertyVanilla() #TODO create a vanilla property as default argument in Individual constructor
    bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(best)))
    geneticPoolDir = os.getcwd()+conf.getDirectory("Genetic Pool")
    bestIndividual.persist(geneticPoolDir + filename)

    #store all the final population, not only the fitest
    population = ga.getPopulation()
    popSize = len(population)
    for pos in range(popSize):
        individual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(population[pos])))
        timestr = time.strftime("%Y%m%d-%H%M%S")
        filename = timestr + "-final" + str(pos) + ".xml"
        individual.persist(geneticPoolDir + filename)

    #do the stats    
    print ga.getStatistics()

if __name__ == "__main__":
   run_main()
