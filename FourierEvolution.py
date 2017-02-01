#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# Evolving a Fourier based joint model
# Recreating the work done by Reda Al-Bahrani et al @ https://sites.google.com/a/u.northwestern.edu/gawalker
# scene ControllerTest_bonusfs for performance comparison
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

import math
import os
import shutil
import time
import xml.etree.cElementTree as ET

from pyevolve import DBAdapters
from pyevolve import G1DList
from pyevolve import GSimpleGA
from pyevolve import Selectors

import Pose
import simulator.Lucy
from configuration.LoadSystemConfiguration  import LoadSystemConfiguration
from simulator.LoadRobotConfiguration import LoadRobotConfiguration

robotConfiguration = LoadRobotConfiguration()
systemConfiguration = LoadSystemConfiguration()
initialPopulationSetted = False
gaEngine = None
NUMBER_GENERATIONS_CONVERGENCE_CRITERIA = 15
max_score = 0
max_score_generation = 0
convergenceCriteria = False
experimentDir = ""
FOURIER_CHROMOSOME_LENGTH = 255
T = 0.5
C_TIMESTEP = 0.05
C_TIMEOFFSET = 10

def setInitialPopulation(ga_engine):


    population = ga_engine.getPopulation()
    popSize = len(population)

    individualCounter = 0

    while individualCounter < popSize:
        adan = population[individualCounter]
        if individualCounter % 2 == 0:
            adan[0] = 0
            adan[1] = math.pi/30
            adan[2] = 0

            adan[3] = 0
            adan[4] = math.pi/30
            adan[5] = 0

            adan[6] = math.pi/60
            adan[7] = -1*math.pi/30
            adan[8] = 0

            adan[9] = 0
            adan[10] = 0
            adan[11] = 0

            adan[12] = 0
            adan[13] = 0
            adan[14] = 0
        else:
            adan[0] = 0
            adan[1] = math.pi/18
            adan[2] = 0

            adan[3] = 0
            adan[4] = 0
            adan[5] = 0

            adan[6] = -1*math.pi/36
            adan[7] = math.pi/12
            adan[8] = 0

            adan[9] = -1*math.pi/12
            adan[10] = math.pi/12
            adan[11] = math.pi

            adan[12] = math.pi/10
            adan[13] = math.pi/60
            adan[14] = -1*math.pi/12
        individualCounter += 1

    global initialPopulationSetted
    initialPopulationSetted = True

def storeExperimentGAparameters():
    file = open(os.path.join(experimentDir,"info.txt"),"w")

    file.write("FOURIER EXPERIMENT:") + "\n")

    file.write("initialPopulationSize = " + systemConfiguration.getProperty("Population size") + "\n")
    file.write("generations = " + systemConfiguration.getProperty("Number of generations") + "\n")

    file.write("selector = " + systemConfiguration.getProperty("Selection operator") + "\n")
    file.write("CrossoverRate = " + systemConfiguration.getProperty("CrossoverRate") + "\n")
    file.write("ElitismReplacement percentage = " + systemConfiguration.getProperty("Elitism replacement percentage") + "\n")
    file.write("Concatenate walk cycles = " + systemConfiguration.getProperty("Concatenate walk cycles?") + "\n")
    file.write("Convergence criteria enable? = " + systemConfiguration.getProperty("Convergence criteria enable?") + "\n")
    file.write("Vrep robot model scene = " + systemConfiguration.getFile("Lucy vrep model") + "\n")

    file.close()

#need to call lucy.executeRawPose to calculate the t
def persist_lucy_time_serie_from_chromosome(chromosome, filename, score):
    SP_C = chromosome[0]
    SP_A = chromosome[1]
    SP_Phi = chromosome[2]

    HR_C = chromosome[3]
    HR_A = chromosome[4]
    HR_Phi = chromosome[5]

    HP_C = chromosome[6]
    HP_A = chromosome[7]
    HP_Phi = chromosome[8]

    K_C = chromosome[9]
    K_A = chromosome[10]
    K_Phi = chromosome[11]

    AP_C = chromosome[12]
    AP_A = chromosome[13]
    AP_Phi = chromosome[14]

    lucy = simulator.Lucy.SimulatedLucy(True)
    pose = {}
    poseNumber = 0
    isUp = lucy.isLucyUp()
    startTime = time.time()

    root = ET.Element("root")
    lucyPersistence = ET.SubElement(root, "Lucy")

    while isUp and poseNumber < FOURIER_CHROMOSOME_LENGTH:
        frame = ET.SubElement(lucyPersistence, "frame")
        frame.set("number", str(poseNumber))
        simTime = time.time() - startTime
        t = simTime - (C_TIMESTEP * C_TIMEOFFSET)

        #Calculate Joint Angles

        #Shoulder Pitch
        pose["L_Shoulder_Pitch"] = SP_C+SP_A*math.sin(2*math.pi*t/T+SP_Phi)
        pose["R_Shoulder_Pitch"] = SP_C+SP_A*math.sin(2*math.pi*t/T+SP_Phi+math.pi)

        #Hip Roll
        pose["L_Hip_Roll"] = HR_C+HR_A*math.sin(2*math.pi*t/T+HR_Phi)
        pose["R_Hip_Roll"] = pose["L_Hip_Roll"]

        #Hip Pitch
        pose["L_Hip_Pitch"] = HP_C+HP_A*math.sin(2*math.pi*t/T+HP_Phi)
        print pose["L_Hip_Pitch"]
        pose["R_Hip_Pitch"] = HP_C+HP_A*math.sin(2*math.pi*t/T+HP_Phi+math.pi)

        #Knee Pitch
        pose["L_Knee"] = K_C+K_A*math.sin(2*math.pi*t/T+K_Phi)
        pose["R_Knee"] = RK_Pos=K_C+K_A*math.sin(2*math.pi*t/T+K_Phi+math.pi)

        #Ankle Pitch
        pose["L_Ankle_Pitch"] = AP_C+AP_A*math.sin(2*math.pi*t/T+AP_Phi)
        pose["R_Ankle_Pitch"] = AP_C+AP_A*math.sin(2*math.pi*t/T+AP_Phi+math.pi)

        #Ankle Roll
        pose["L_Ankle_Roll"] = pose["L_Hip_Roll"]
        pose["R_Ankle_Roll"] = pose["L_Ankle_Roll"]

        newPose = Pose.Pose(pose)
        lucy.executeRawPose(newPose)

        for joint in robotConfiguration.getJointsName():
            xmlJoint = ET.SubElement(frame, joint)
            angle = pose[joint]
            xmlJoint.set("angle", str(angle))

        poseNumber += 1

    fitness = lucy.getFitness(FOURIER_CHROMOSOME_LENGTH, 1)
    if fitness == score:
        print "stored fitness correspond with trained"
    else:
        print "stored fitness DOESN'T correspond with trained"

    lucy.stopLucy()

    TFTparameters = ET.SubElement(lucyPersistence, "TFT parameters")

    TFTparameters.set("SP_C", str(SP_C))
    TFTparameters.set("SP_A", str(SP_A))
    TFTparameters.set("SP_Phi", str(SP_Phi))

    TFTparameters.set("HR_C", str(HR_C))
    TFTparameters.set("HR_A", str(HR_A))
    TFTparameters.set("HR_A", str(HR_A))

    TFTparameters.set("HP_C", str(HP_C))
    TFTparameters.set("HP_A", str(HP_A))
    TFTparameters.set("HP_Phi", str(HP_Phi))

    TFTparameters.set("K_C", str(K_C))
    TFTparameters.set("K_A", str(K_A))
    TFTparameters.set("K_Phi", str(K_Phi))

    TFTparameters.set("AP_C", str(AP_C))
    TFTparameters.set("AP_A", str(AP_A))
    TFTparameters.set("AP_Phi", str(AP_Phi))

    tree = ET.ElementTree(root)
    tree.write(filename)


# This function is the evaluation function
# Takes a chromosome representing the Truncated Fourier Serie parameters and generates the value of each joint.
# Each calculated Pose is executed in vrep simulator
def eval_func(chromosome):
    if not initialPopulationSetted:
        setInitialPopulation(gaEngine)

    SP_C = chromosome[0]
    SP_A = chromosome[1]
    SP_Phi = chromosome[2]

    HR_C = chromosome[3]
    HR_A = chromosome[4]
    HR_Phi = chromosome[5]

    HP_C = chromosome[6]
    HP_A = chromosome[7]
    HP_Phi = chromosome[8]

    K_C = chromosome[9]
    K_A = chromosome[10]
    K_Phi = chromosome[11]

    AP_C = chromosome[12]
    AP_A = chromosome[13]
    AP_Phi = chromosome[14]


    lucy = simulator.Lucy.SimulatedLucy(True)
    pose = {}
    poseNumber = 0
    isUp = lucy.isLucyUp()
    startTime = time.time()

    while isUp and poseNumber < FOURIER_CHROMOSOME_LENGTH:
        simTime = time.time() - startTime
        t = simTime - (C_TIMESTEP * C_TIMEOFFSET)

        #Calculate Joint Angles

        #Shoulder Pitch
        pose["L_Shoulder_Pitch"] = SP_C+SP_A*math.sin(2*math.pi*t/T+SP_Phi)
        pose["R_Shoulder_Pitch"] = SP_C+SP_A*math.sin(2*math.pi*t/T+SP_Phi+math.pi)

        #Hip Roll
        pose["L_Hip_Roll"] = HR_C+HR_A*math.sin(2*math.pi*t/T+HR_Phi)
        pose["R_Hip_Roll"] = pose["L_Hip_Roll"]

        #Hip Pitch
        pose["L_Hip_Pitch"] = HP_C+HP_A*math.sin(2*math.pi*t/T+HP_Phi)
        pose["R_Hip_Pitch"] = HP_C+HP_A*math.sin(2*math.pi*t/T+HP_Phi+math.pi)

        #Knee Pitch
        pose["L_Knee"] = K_C+K_A*math.sin(2*math.pi*t/T+K_Phi)
        pose["R_Knee"] = RK_Pos=K_C+K_A*math.sin(2*math.pi*t/T+K_Phi+math.pi)

        #Ankle Pitch
        pose["L_Ankle_Pitch"] = AP_C+AP_A*math.sin(2*math.pi*t/T+AP_Phi)
        pose["R_Ankle_Pitch"] = AP_C+AP_A*math.sin(2*math.pi*t/T+AP_Phi+math.pi)

        #Ankle Roll
        pose["L_Ankle_Roll"] = pose["L_Hip_Roll"]
        pose["R_Ankle_Roll"] = pose["L_Ankle_Roll"]

        newPose = Pose.Pose(pose)
        lucy.executeRawPose(newPose)
        poseNumber += 1
        isUp = lucy.isLucyUp()
        #TODO wait the amount of time needed to syncronize with lucy frame rate

    fitness = lucy.getFitness(FOURIER_CHROMOSOME_LENGTH, 1)
    lucy.stopLucy()
    return fitness


# This function is the termination criteria for the algorithm
def ConvergenceCriteria(ga_engine):
    global convergenceCriteria
    return convergenceCriteria

def generationCallback(ga_engine):
    # persist best individual at the moment
    geneticPoolDir = os.getcwd() + systemConfiguration.getDirectory("Genetic Pool")
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

    persist_lucy_time_serie_from_chromosome(best, filename, score)

    ga_engine.getDBAdapter().commit()

    print "generation executed!, best fit of generation: ", score, "fittest: ", max_score, "reached in generation: ", max_score_generation

    return False


def run_main():
    initialPopulationSize = int(systemConfiguration.getProperty("Population size"))
    generations = int(systemConfiguration.getProperty("Number of generations"))
    # Genome instance

    genome = G1DList.G1DList(15)
    genome.setParams(rangemin=-50, rangemax=50)
    # The evaluator function (objective function)
    genome.evaluator.set(eval_func)

    # Genetic Algorithm Instance
    ga = GSimpleGA.GSimpleGA(genome)
    ga.setGenerations(generations)
    ga.setPopulationSize(initialPopulationSize)

    if systemConfiguration.getProperty("Selection operator") == "Selectors.GRankSelector" :
        ga.selector.set(Selectors.GRankSelector)
    elif systemConfiguration.getProperty("Selection operator") == "Selectors.GTournamentSelector" :
        ga.selector.set(Selectors.GTournamentSelector)
    elif systemConfiguration.getProperty("Selection operator") == "Selectors.GRouletteWheel" :
        ga.selector.set(Selectors.GRouletteWheel)
    elif systemConfiguration.getProperty("Selection operator") == "Selectors.GUniformSelector" :
        ga.selector.set(Selectors.GUniformSelector)

    '''For crossover probability, maybe it is the ratio of next generation population born by crossover operation.
    While the rest of population...maybe by previous selection or you can define it as best fit survivors'''
    ga.setCrossoverRate(float(systemConfiguration.getProperty("CrossoverRate")))

    elitism = float(systemConfiguration.getProperty("Elitism replacement percentage")) > 0
    ga.setElitism(True)

    '''Set the number of best individuals to copy to the next generation on the elitism'''
    if elitism:
        numberIndividualsForNextGen = int(initialPopulationSize * float(systemConfiguration.getProperty("Elitism replacement percentage")))
        ga.setElitismReplacement(numberIndividualsForNextGen)

    if int(systemConfiguration.getProperty("Convergence criteria enable?")) == True:
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

    #store the final population
    population = ga.getPopulation()
    popSize = len(population)
    for pos in range(popSize):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        filename = "final-" + str(pos) + "-" + timestr + ".xml"
        filename = os.path.join(experimentDir,filename)
        chromosomeToPersist = population[pos]
        score = chromosomeToPersist.getRawScore()
        persist_lucy_time_serie_from_chromosome(chromosomeToPersist, filename, score)

    #ga.getDBAdapter().commit()

    shutil.copy2('pyevolve.db', experimentDir)
    shutil.copy2(systemConfiguration.getProperty("System Log"), experimentDir)

    #do the stats
    print ga.getStatistics()


if __name__ == "__main__":
    run_main()







