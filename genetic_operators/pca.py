#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Patricia Polero
# Damián Ferraro
# CENUR LN/UDELAR
#
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
#
# PCA interpolation of the pose
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

from datatypes.DTIndividualProperty             import DTIndividualProperty, DTIndividualPropertyCMUDaz, DTIndividualPropertyVanilla, DTIndividualPropertyBaliero, DTIndividualPropertyVanillaEvolutive
from datatypes.DTIndividualGeneticMaterial      import DTIndividualGeneticMaterial, DTIndividualGeneticTimeSerieFile, DTIndividualGeneticMatrix
from configuration.LoadSystemConfiguration      import LoadSystemConfiguration
from simulator.LoadRobotConfiguration           import LoadRobotConfiguration
from Individual                                 import Individual

import os 
import glob 
import numpy as np
from scipy import linalg

MIN_EIGEN_VALUE = 1.0

def jointsNearMutationPoint(mutationJoint, nameIndexMapping):
    returnList = []

    if mutationJoint == "R_Elbow_Yaw":
        returnList.append(nameIndexMapping["R_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["L_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["L_Elbow_Yaw"])

    if mutationJoint == "L_Elbow_Yaw":
        returnList.append(nameIndexMapping["L_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["R_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["R_Elbow_Yaw"])

    if mutationJoint == "R_Shoulder_Yaw":
        returnList.append(nameIndexMapping["L_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["L_Shoulder_Pitch"])
        returnList.append(nameIndexMapping["R_Shoulder_Pitch"])

    if mutationJoint == "L_Shoulder_Yaw":
        returnList.append(nameIndexMapping["R_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["R_Shoulder_Pitch"])
        returnList.append(nameIndexMapping["L_Shoulder_Pitch"])

    if mutationJoint == "R_Shoulder_Pitch":
        returnList.append(nameIndexMapping["R_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["L_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["L_Shoulder_Pitch"])

    if mutationJoint == "L_Shoulder_Pitch":
        returnList.append(nameIndexMapping["L_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["R_Shoulder_Yaw"])
        returnList.append(nameIndexMapping["R_Shoulder_Pitch"])

    if mutationJoint == "R_Hip_Yaw":
        returnList.append(nameIndexMapping["L_Hip_Yaw"])

    if mutationJoint == "l_Hip_Yaw":
        returnList.append(nameIndexMapping["R_Hip_Yaw"])

    if mutationJoint == "R_Hip_Roll":
        returnList.append(nameIndexMapping["L_Hip_Roll"])
        returnList.append(nameIndexMapping["R_Hip_Pitch"])
        returnList.append(nameIndexMapping["R_Knee"])
        #returnList.append(nameIndexMapping["L_Hip_Pitch"])
        returnList.append(nameIndexMapping["L_Knee"])

    if mutationJoint == "L_Hip_Roll":
        returnList.append(nameIndexMapping["R_Hip_Roll"])
        returnList.append(nameIndexMapping["L_Hip_Pitch"])
        returnList.append(nameIndexMapping["L_Knee"])
        #returnList.append(nameIndexMapping["R_Hip_Pitch"])
        returnList.append(nameIndexMapping["R_Knee"])

    if mutationJoint == "R_Hip_Pitch":
        returnList.append(nameIndexMapping["L_Hip_Pitch"])
        returnList.append(nameIndexMapping["R_Hip_Roll"])
        #returnList.append(nameIndexMapping["L_Hip_Roll"])
        returnList.append(nameIndexMapping["R_Knee"])
        returnList.append(nameIndexMapping["L_Knee"])

    if mutationJoint == "L_Hip_Pitch":
        returnList.append(nameIndexMapping["R_Hip_Pitch"])
        returnList.append(nameIndexMapping["L_Hip_Roll"])
        #returnList.append(nameIndexMapping["R_Hip_Roll"])
        returnList.append(nameIndexMapping["L_Knee"])
        returnList.append(nameIndexMapping["R_Knee"])

    if mutationJoint == "R_Knee":
        returnList.append(nameIndexMapping["L_Knee"])
        returnList.append(nameIndexMapping["R_Hip_Pitch"])
        returnList.append(nameIndexMapping["L_Hip_Pitch"])
        returnList.append(nameIndexMapping["R_Ankle_Pitch"])
        #returnList.append(nameIndexMapping["L_Ankle_Pitch"])

    if mutationJoint == "L_Knee":
        returnList.append(nameIndexMapping["R_Knee"])
        returnList.append(nameIndexMapping["L_Hip_Pitch"])
        returnList.append(nameIndexMapping["R_Hip_Pitch"])
        returnList.append(nameIndexMapping["L_Ankle_Pitch"])
        #returnList.append(nameIndexMapping["R_Ankle_Pitch"])

    if mutationJoint == "R_Ankle_Pitch":
        returnList.append(nameIndexMapping["L_Ankle_Pitch"])
        returnList.append(nameIndexMapping["L_Knee"])
        returnList.append(nameIndexMapping["R_Knee"])

    if mutationJoint == "L_Ankle_Pitch":
        returnList.append(nameIndexMapping["R_Ankle_Pitch"])
        returnList.append(nameIndexMapping["R_Knee"])
        returnList.append(nameIndexMapping["L_Knee"])

    #Ankle_Roll is not use due to incertanty in the calculation
    return returnList

#mymatrix is a numpy matrix
def swapCols(mymatrix, frm, to):
    mymatrix[:,[frm, to]] = mymatrix[:,[to, frm]]

def poseInterpolationWithPCA(geneticMatrix, mutationJointID):
    robotConfig = LoadRobotConfiguration()
    joints = robotConfig.getJointsName()
    propCMUDaz = DTIndividualPropertyCMUDaz()
    nameIndexMapping = {}
    walk = Individual(propCMUDaz, DTIndividualGeneticMatrix(geneticMatrix))
    mymatrix = [[0 for j in xrange(18)] for i in xrange(len(geneticMatrix))]
    for i in xrange(len(geneticMatrix)):
        for joint in joints:
                #print "pose number: " + str(i) + " joint: " + j + " value: " + str(geneticMatrix[i][walk.getJointMatrixIDFromName(j)])
                IDJoint = walk.getJointMatrixIDFromName(joint)
                mymatrix[i][IDJoint] = geneticMatrix[i][IDJoint]
                nameIndexMapping[joint]=IDJoint
                if IDJoint == 0:
                    nameJointZero = joint
                if IDJoint == mutationJointID:
                    jointNameToMutate = joint

    calcmatrix = np.asmatrix(mymatrix)
    swapCols(calcmatrix, mutationJointID, 0)

    nameIndexMapping[jointNameToMutate] = 0
    nameIndexMapping[nameJointZero] = mutationJointID



    idJointsToApplyPCA = jointsNearMutationPoint(jointNameToMutate, nameIndexMapping)

    iterCounter = -1
    for idJoint in idJointsToApplyPCA:
        swapCols(calcmatrix, idJoint, iterCounter)
        iterCounter -= 1

    TotalAngles=calcmatrix.shape

    #Aca pasar para el inicio de la matriz la columna correspondiente al angulo que sufrio una mutacion y pasar para el final de la matriz los angulos que queremos recuperar. Estos deberian estar ya previamente estipulados en una tabla (segun cual muta, cuales se recuperan)

    # 1. CENTRAR LA MATRIZ

    media = []
    calcmatrixNorm = np.zeros_like(calcmatrix)
    for i in range (TotalAngles[1]):
        media.append (np.mean(calcmatrix[:,i]))
        calcmatrixNorm[:,i]=(calcmatrix[:,i]-media[i])

    #print np.mean (calcmatrixNorm[:,1])
    # Si la matriz esta normalizada la media de cada columna tiene que ser cero.

    # 2. MATRIX DE COVARIANZA

    VarMatrix=np.cov(calcmatrixNorm, rowvar=0, bias=1)

    #print VarMatrix
    #Tiene que ser simetrica con 1 en la diagonal.

    # 3. VALORES Y VECTORES PROPIOS

    U=linalg.eigh(VarMatrix, type=3)
    Uval= U[0]
    Uvec=np.asmatrix(U[1])

    print Uval
    #print Uval[-1]*(Uvec[:,-1])
    #print VarMatrix*(Uvec[:,-1])
    #Se tiene que cumplir esta igualdad.

    # 4. SELECCIONAR LOS VECTORES PROPIOS ASOCIADOS A VALORES PROPIOS NULOS

    NumberValNul=np.asmatrix(np.where(Uval<MIN_EIGEN_VALUE))
    NumberValNul=NumberValNul.shape
    MatrixUnul=np.zeros((TotalAngles[1],NumberValNul[1]))
    MatrixUnul=np.asmatrix(MatrixUnul)

    for i in range(TotalAngles[1]):
        if abs (Uval[i]) < MIN_EIGEN_VALUE:
            MatrixUnul[:,i]=Uvec[:,i]

    # 5. RESOLUCION DEL SISTEMA

    #   5.1)ESCALERIZAR LA MATRIZ DE VECT P. ASOCIADOS A VAL P. NULOS

    q, escalerizada = np.linalg.qr(MatrixUnul.T)
    triangINF=np.asmatrix(escalerizada.T)
    #Tiene que ser triangular inferior cuadrada con la misma cantidad de columnas que de angulos

    #   5.2)BUSCAR LOS ANGULOS PRINCIPALES

    #Me quedo con la parte de la matriz que me sirve para despejar los otros angulos (no quedan ni los escalones dobles, ni las ultimas filas).

    indexlist=[]
    PREMatrix=np.asmatrix(np.zeros((NumberValNul[1],NumberValNul[1])))
    i=0
    for j in range(NumberValNul[1]):
        while abs (triangINF[i,j]) < 0.000001:
            i=i+1
        indexlist.append(i)
        PREMatrix[j,:]=triangINF[i,:]
    indexlist=np.asmatrix(indexlist)
    print "indexlist"
    print indexlist


    # LOS QUE NO ESTAN EN INDEXLIST SON LOS PRINCIPALES, SI SON LOS ULTIMOS, ES PORQUE SON ESOS O CUALQUIERA (PERO ESA ES LA CANTIDAD MINIMA QUE PRECISO)
    #print PREMatrix.shape
    #Tiene que ser cuadrada con el mismo numero de columnas que de vectores propios asociados a val propios nulos.

    InvPREMatrix = np.linalg.inv(PREMatrix)
    a=np.dot(InvPREMatrix,-1)
    InvPREMatrixTriang = np.dot(triangINF,a)

    # 6) MATRIZ PARA RECUPERAR LOS ANGULOS A PARTIR DE LOS PRINCIPALES

    NOTindexlist=[]
    RecoveryMatrix=np.asmatrix(np.zeros(((TotalAngles[1]-NumberValNul[1]),NumberValNul[1])))
    k=0
    for i in range (TotalAngles[1]):
        if not (i in indexlist):
            RecoveryMatrix[k,:]=InvPREMatrixTriang[i,:]
            k=k+1
            NOTindexlist.append(i)

    print "los angulos principales son las columnas: "
    print NOTindexlist
    #print RecoveryMatrix
    #print RecoveryMatrix.shape

    ############################################################
    # Ingresa la matriz con cualquier numero de filas (poses) y solo con los angulos principales

    # 1) MATRIZ ORIGINAL SOLO CON LOS ANGULOS PRINCIPALES

    PrincMatrix=np.delete(calcmatrixNorm,indexlist,1)

    #print TotalAngles[1]-NumberValNul[1]
    #print PrincMatrix.shape
    #Estos dos valores tienen que ser iguales

    # 2) RECUPERACION DE TODOS LOS ANGULOS A PARTIR DE LOS PRINCIPALES

    NewAngles= np.dot(PrincMatrix,RecoveryMatrix)
    FinalAngles=np.hstack((NewAngles,PrincMatrix)) ##Esto solo sirve si siempre los ang principales son las ultimas columnas
    print FinalAngles.shape

    #TODO reconstruct the matrix swaping what it was changed