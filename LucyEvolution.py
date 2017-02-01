#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Lucy evolution main program
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


import glob
import os
import shutil
import time

from pyevolve import DBAdapters
from pyevolve import G2DList
from pyevolve import GSimpleGA
from pyevolve import Mutators
from pyevolve import Selectors

import configuration.constants as sysConstants
from Individual                                 import Individual
from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
from datatypes.DTGenomeFunctions import  DTGenomeFunctions
from datatypes.DTIndividualGeneticMaterial      import DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix, DTIndividualGeneticMatrixWalk
from datatypes.DTIndividualProperty             import DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero, DTIndividualPropertyVanillaEvolutive
from genetic_operators import crossovers, mutators

initialPopulationSetted = False
gaEngine = None
NUMBER_GENERATIONS_CONVERGENCE_CRITERIA = 15
max_score = 0
max_score_generation = 0
convergenceCriteria = False
experimentDir = ""

def storeExperimentGAparameters():
    conf = LoadSystemConfiguration()
    file = open(os.path.join(experimentDir,"info.txt"),"w")

    file.write("RANDOM INITIAL POPULATION EXPERIMENT:" + "\n")

    file.write("initialPopulationSize = " + conf.getProperty("Population size") + "\n")
    file.write("generations = " + conf.getProperty("Number of generations") + "\n")
    file.write("genome.crossover = " + conf.getProperty("Crossover operator") + "\n")
    file.write("genome.mutator = " + conf.getProperty("Mutator operator") + "\n")
    
    file.write("MutationRate = " + conf.getProperty("MutationRate") + "\n")
    
    file.write("selector = " + conf.getProperty("Selection operator") + "\n")
    file.write("CrossoverRate = " + conf.getProperty("CrossoverRate") + "\n")
    file.write("ElitismReplacement percentage = " + conf.getProperty("Elitism replacement percentage") + "\n")
    file.write("Concatenate walk cycles = " + conf.getProperty("Concatenate walk cycles?") + "\n")
    file.write("concatenate external cycle file = " + conf.getProperty("concatenate external cycle file?") + "\n")
    file.write("Convergence criteria enable? = " + conf.getProperty("Convergence criteria enable?") + "\n")
    file.write("Vrep robot model scene = " + conf.getFile("Lucy vrep model") + "\n")

    file.close()

def getPopulationAverage(population):
    average = 0
    for p in population:
        score = p.getRawScore()
        average = average + score
    return average/len(population)


def setInitialPopulation(ga_engine):

    propCMUDaz = DTIndividualPropertyCMUDaz()
    propVanilla = DTIndividualPropertyVanilla()
    balieroProp = DTIndividualPropertyBaliero()

    conf = LoadSystemConfiguration()

    lucyCycles = os.getcwd()+conf.getDirectory("Lucy evolved walk cycles Files")
    CMUxmlDir = os.getcwd()+conf.getDirectory("Transformed CMU mocap Files")
    GAwalkDir = os.getcwd()+conf.getDirectory("GAWalk Files")
    UIBLHDir = os.getcwd()+conf.getDirectory("UIBLH mocap Files")
    BalieroDir = os.getcwd()+conf.getDirectory("Baliero transformed walk Files")
    ADHOCDir = os.getcwd()+conf.getDirectory("ADHOC Files")
    geneticPoolDir = os.getcwd()+conf.getDirectory("Genetic Pool")
    randomDir = os.getcwd()+conf.getDirectory("Random Individual Files")
    
    population = ga_engine.getPopulation()
    popSize = len(population)

    individualCounter = 0
    walk = Individual(propVanilla, DTIndividualGeneticMatrix()) #dummy individual to initialise the simulator and enable the time step configuration
    walk.execute()
    print "please set the proper time step in vrep"


    dtgenoma = DTGenomeFunctions()


    #the random initia population created is replaced by the imitation motion capture database
    if individualCounter < popSize:
        for filename in glob.glob(os.path.join(randomDir, '*.xml')):
            if individualCounter < popSize:
                print individualCounter, " individuals processed!"
                print 'inserting individual: ' + filename + " into the initial population"
                walk = Individual(propVanilla, DTIndividualGeneticTimeSerieFile(filename))
                teacherGeneticMatrix = walk.getGenomeMatrix()
                adan = population[individualCounter]
                adanIndividualLength=dtgenoma.getIndividualLength(adan)
                adanJointLength=dtgenoma.getIndividualFrameLength(adan)
                for i in xrange(adanIndividualLength):
                    for j in xrange(adanJointLength):
                        if i < len(teacherGeneticMatrix): #if the fixed gnoma representation size is less than the teacher size
                            adan.setItem(i,j,teacherGeneticMatrix[i][j])
                        else:
                            #put a sentinel joint value to mark the end of the individual
                            adan.setItem(i,j,sysConstants.JOINT_SENTINEL)
                population[individualCounter]=adan
                individualCounter = individualCounter + 1
            else:
                break
    '''
    if individualCounter < popSize:
        for filename in glob.glob(os.path.join(lucyCycles, '*.xml')):
            if individualCounter < popSize:
                print individualCounter, " individuals processed!"
                print 'inserting individual: ' + filename + " into the initial population"
                walk = Individual(propVanilla, DTIndividualGeneticTimeSerieFile(filename))
                teacherGeneticMatrix = walk.getGenomeMatrix()
                adan = population[individualCounter]
                adanIndividualLength=dtgenoma.getIndividualLength(adan)
                adanJointLength=dtgenoma.getIndividualFrameLength(adan)
                for i in xrange(adanIndividualLength):
                    for j in xrange(adanJointLength):
                        if i < len(teacherGeneticMatrix): #if the fixed gnoma representation size is less than the teacher size
                            adan.setItem(i,j,teacherGeneticMatrix[i][j])
                        else:
                            #put a sentinel joint value to mark the end of the individual
                            adan.setItem(i,j,sysConstants.JOINT_SENTINEL)
                population[individualCounter]=adan
                individualCounter = individualCounter + 1
            else:
                break
    '''

    global initialPopulationSetted
    initialPopulationSetted = True

def chromosomeToLucyGeneticMatrix(chromosome): #TODO encapsulate this in a helper class
    geneticMatrix = [[chromosome[i][j] for j in xrange(chromosome.getWidth())] for i in xrange(chromosome.getHeight())]
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
        #if the score doesn't improve in NUMBER_GENERATIONS_CONVERGENCE_CRITERIA generations then there is no reason to continue and we have reach a convergence
        if gen - max_score_generation > NUMBER_GENERATIONS_CONVERGENCE_CRITERIA:
            convergenceCriteria = True

    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = str(score) + "-" + timestr + "-" + str(gen) + ".xml"

    #at generation 0 is the firs time to create de directory and store the GA parameters
    if gen == 0:
        global experimentDir
        experimentDir = geneticPoolDir + timestr
        os.mkdir(experimentDir)
        storeExperimentGAparameters()

    prop = DTIndividualPropertyVanilla()
    bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(best)))
    bestIndividual.persist(os.path.join(experimentDir, filename))
    ga_engine.getDBAdapter().commit()

    #population = ga_engine.getPopulation()
    #popSize = len(population)
    print "generation executed!, best fit of generation: ", score, "fittest: ", max_score, "reached in generation: ", max_score_generation

    return False

# This function is the evaluation function
def eval_func(chromosome):
    conf = LoadSystemConfiguration()
    if not initialPopulationSetted:
        setInitialPopulation(gaEngine)

    prop = DTIndividualPropertyVanillaEvolutive()
    preCycleLength = 0

    if int(conf.getProperty("Concatenate walk cycles?")):
        embryo = DTIndividualGeneticMatrixWalk(chromosomeToLucyGeneticMatrix(chromosome))
        embryoCycleLength = (embryo.getLength() - preCycleLength) / int(conf.getProperty("Concatenate walk cycles?"))

    else:
        embryo = DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(chromosome))
        embryoCycleLength = embryo.getLength() - preCycleLength


    #embryoLength = newEmbryo.getLength()
    individual = Individual(prop, embryo)
    individual.setPrecycleLength(preCycleLength)

    individual.setCycleLength(embryoCycleLength)
    print "el cycle length seteado es: ", embryoCycleLength

    ##print "precyclelength:  ", preCycleLength
    ##print "cyclelength:  ", embryoCycleLength
    #individual.setLength(embryoLength)
    fitness = individual.execute() #return the fitness resulting from the simulator execution

    if int(conf.getProperty("re-evaluate fittest?"))==True:
        if fitness > max_score: #is really a better fitness ?
            candidateFitness = fitness
            fitness = individual.execute()
            print "candidateFitness: ", candidateFitness, "fitness: ", fitness
            while abs(candidateFitness-fitness) > 0.01:
                candidateFitness=fitness
                fitness = individual.execute()
                print "candidateFitness: ", candidateFitness, "fitness: ", fitness
            #the candidateFitness was validated!
            fitness = candidateFitness
    return fitness

# This function is the termination criteria for the algorithm
def ConvergenceCriteria(ga_engine):
    global convergenceCriteria
    return convergenceCriteria

def run_main():
    conf = LoadSystemConfiguration()
    initialPopulationSize = int(conf.getProperty("Population size"))
    generations = int(conf.getProperty("Number of generations"))
    # Genome instance
    framesQty = sysConstants.GENOMA_MAX_LENGTH
    genome = G2DList.G2DList(framesQty, 18)
    genome.setParams(rangemin=0, rangemax=360)
    genome.setParams(gauss_sigma=sysConstants.MUTATION_SIGMA, gauss_mu=0)

    # The evaluator function (objective function)
    genome.evaluator.set(eval_func)
    if conf.getProperty("Crossover operator") == "crossovers.G2DListCrossoverSingleNearHPoint":
        genome.crossover.set(crossovers.G2DListCrossoverSingleNearHPoint)

    #genome.crossover.set(crossovers.G2DListCrossoverSingleNearHPointImprove)
    #genome.crossover.set(crossovers.G2DListCrossoverSingleHPoint)
    
    # Genetic Algorithm Instance
    ga = GSimpleGA.GSimpleGA(genome)
    ga.setGenerations(generations)
    ga.setPopulationSize(initialPopulationSize)

    #genome.mutator.set(Mutators.G2DListMutatorIntegerRange)
    if conf.getProperty("Mutator operator") == "mutators.G2DListMutatorRealGaussianSpline":
        genome.mutator.set(mutators.G2DListMutatorRealGaussianSpline)
    elif conf.getProperty("Mutator operator") == "Mutators.G2DListMutatorRealGaussianGradient":
        genome.mutator.set(Mutators.G2DListMutatorRealGaussianGradient)
    
    ga.setMutationRate(float(conf.getProperty("MutationRate")))
    
    if conf.getProperty("Selection operator") == "Selectors.GRankSelector" :
        ga.selector.set(Selectors.GRankSelector)
    elif conf.getProperty("Selection operator") == "Selectors.GTournamentSelector" :
        ga.selector.set(Selectors.GTournamentSelector)
    elif conf.getProperty("Selection operator") == "Selectors.GRouletteWheel" :
        ga.selector.set(Selectors.GRouletteWheel)
    elif conf.getProperty("Selection operator") == "Selectors.GUniformSelector" :
        ga.selector.set(Selectors.GUniformSelector)

    '''For crossover probability, maybe it is the ratio of next generation population born by crossover operation. 
    While the rest of population...maybe by previous selection or you can define it as best fit survivors'''
    ga.setCrossoverRate(float(conf.getProperty("CrossoverRate")))

    elitism = float(conf.getProperty("Elitism replacement percentage")) > 0
    ga.setElitism(True)
    '''Set the number of best individuals to copy to the next generation on the elitism'''
    if elitism:
        numberIndividualsForNextGen = int(initialPopulationSize*float(conf.getProperty("Elitism replacement percentage")))
        ga.setElitismReplacement(numberIndividualsForNextGen)

    if int(conf.getProperty("Convergence criteria enable?"))==True:
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
    # frequency of every generation
    ga.evolve(freq_stats=0)

    # Best individual
    best = ga.bestIndividual()
    score = best.getRawScore()
    timestr = time.strftime("%Y%m%d-%H%M%S")
    filename = str(score) + "-" + timestr + "-" + str(generations) + ".xml"
    prop = DTIndividualPropertyVanilla()
    bestIndividual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(best)))

    bestIndividual.persist(os.path.join(experimentDir,filename))

    #store all the final population, not only the fitest
    population = ga.getPopulation()
    popSize = len(population)
    for pos in range(popSize):
        individual = Individual(prop, DTIndividualGeneticMatrix(chromosomeToLucyGeneticMatrix(population[pos])))
        timestr = time.strftime("%Y%m%d-%H%M%S")
        filename = "final-" + str(pos) + "-" + timestr + ".xml"
        individual.persist(os.path.join(experimentDir, filename))
    #ga.getDBAdapter().commit()
    
    shutil.copy2('pyevolve.db', experimentDir)
    shutil.copy2(conf.getProperty("System Log"), experimentDir)
    
    #do the stats    
    print ga.getStatistics()


if __name__ == "__main__":
    run_main()
