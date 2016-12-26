#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# 
# Program to convert Marcelo Baliero and Gerardo Pias 
# poses time series into Lucy time series
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License,  or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not,  write to the Free Software
# Foundation,  Inc.,  51 Franklin St,  Fifth Floor,  Boston,  MA  02110-1301  USA


import xml.etree.cElementTree as ET

from Pose                             import Pose
from simulator.AXAngle                import AXAngle
from simulator.LoadRobotConfiguration import LoadRobotConfiguration
from simulator.Lucy                   import SimulatedLucy

posesVect = {}
angle = AXAngle()
BalieroLucyMapping = {}
newPose = Pose()
robotConfig = LoadRobotConfiguration()
lucy = SimulatedLucy(True)


root = ET.Element("root")
lucyPersistence = ET.SubElement(root,  "Lucy")
configuration = LoadRobotConfiguration()


BalieroLucyMapping['L_Shoulder_Pitch']=2
BalieroLucyMapping['R_Shoulder_Pitch']=1
BalieroLucyMapping['L_Shoulder_Yaw']=4
BalieroLucyMapping['R_Shoulder_Yaw']=3
BalieroLucyMapping['L_Elbow_Yaw']=6
BalieroLucyMapping['R_Elbow_Yaw']=5
BalieroLucyMapping['R_Hip_Yaw']=7
BalieroLucyMapping['L_Hip_Yaw']=8
BalieroLucyMapping['L_Hip_Roll']=10
BalieroLucyMapping['R_Hip_Roll']=9
BalieroLucyMapping['L_Hip_Pitch']=12
BalieroLucyMapping['R_Hip_Pitch']=11
BalieroLucyMapping['L_Knee']=14
BalieroLucyMapping['R_Knee']=13
BalieroLucyMapping['L_Ankle_Pitch']=18
BalieroLucyMapping['R_Ankle_Pitch']=17
BalieroLucyMapping['L_Ankle_Roll']=16
BalieroLucyMapping['R_Ankle_Roll']=15

mirrorPitch = [2, 12,  14,  18]


#fsr

posesVect[0] = [235, 788, 279, 744, 462, 561, 358, 666, 501, 510, 342, 681, 242, 782, 646, 372, 501, 490]
posesVect[1] = [235, 788, 279, 744, 462, 561, 358, 666, 495, 504, 345, 679, 247, 778, 643, 374, 495, 484]
posesVect[2] = [235, 788, 279, 744, 462, 561, 358, 666, 490, 499, 348, 677, 253, 773, 640, 376, 490, 479]
posesVect[3] = [235, 788, 279, 744, 462, 561, 358, 666, 486, 495, 351, 674, 260, 767, 637, 379, 486, 475]
posesVect[4] = [235, 788, 279, 744, 462, 561, 358, 666, 479, 502, 344, 671, 245, 761, 644, 382, 482, 472]
posesVect[5] = [235, 788, 279, 744, 462, 561, 358, 666, 468, 520, 327, 669, 212, 757, 661, 384, 479, 470]
posesVect[6] = [235, 788, 279, 744, 462, 561, 358, 666, 463, 529, 320, 668, 197, 755, 668, 385, 477, 469]
posesVect[7] = [236, 789, 279, 744, 462, 561, 358, 666, 468, 520, 298, 670, 216, 756, 637, 377, 479, 470]
posesVect[8] = [221, 774, 279, 744, 462, 561, 358, 666, 479, 502, 305, 666, 258, 759, 612, 378, 482, 472]
posesVect[9] = [206, 759, 279, 744, 462, 561, 358, 666, 486, 495, 310, 668, 279, 764, 596, 374, 486, 475]
posesVect[10] = [193, 746, 279, 744, 462, 561, 358, 666, 490, 499, 305, 671, 272, 771, 599, 371, 490, 479]
posesVect[11] = [182, 735, 279, 744, 462, 561, 358, 666, 495, 504, 303, 670, 263, 775, 606, 366, 495, 484]
posesVect[12] = [174, 727, 279, 744, 462, 561, 358, 666, 501, 510, 304, 666, 254, 777, 615, 360, 501, 490]
posesVect[13] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]


#pause

posesVect[14] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[15] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[16] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[17] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[18] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[19] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[20] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[21] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[22] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
posesVect[23] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]

#fml

posesVect[24] = [169, 722, 279, 744, 462, 561, 358, 666, 513, 522, 302, 660, 246, 769, 621, 348, 513, 532]
posesVect[25] = [166, 719, 279, 744, 462, 561, 358, 666, 518, 527, 308, 651, 248, 761, 625, 348, 518, 537]
posesVect[26] = [167, 720, 279, 744, 462, 561, 358, 666, 519, 533, 314, 650, 252, 761, 628, 346, 523, 542]
posesVect[27] = [171, 724, 279, 744, 462, 561, 358, 666, 513, 541, 321, 662, 257, 778, 629, 341, 535, 547]
posesVect[28] = [179, 732, 279, 744, 462, 561, 358, 666, 504, 550, 327, 682, 262, 801, 644, 338, 561, 551]
posesVect[29] = [189, 742, 279, 744, 462, 561, 358, 666, 496, 556, 332, 704, 266, 820, 646, 342, 562, 553]
posesVect[30] = [201, 754, 279, 744, 462, 561, 358, 666, 493, 558, 335, 724, 267, 827, 648, 354, 563, 554]
posesVect[31] = [215, 768, 279, 744, 462, 561, 358, 666, 496, 556, 337, 736, 266, 820, 646, 374, 562, 553]
posesVect[32] = [230, 783, 279, 744, 462, 561, 358, 666, 504, 550, 338, 740, 262, 801, 646, 396, 561, 551]
posesVect[33] = [246, 799, 279, 744, 462, 561, 358, 666, 513, 541, 339, 737, 257, 778, 647, 416, 535, 547]
posesVect[34] = [260, 813, 279, 744, 462, 561, 358, 666, 519, 533, 340, 732, 252, 761, 654, 428, 523, 542]
posesVect[35] = [274, 827, 279, 744, 462, 561, 358, 666, 518, 527, 343, 730, 248, 761, 660, 427, 518, 537]
posesVect[36] = [285, 838, 279, 744, 462, 561, 358, 666, 513, 522, 347, 730, 246, 769, 666, 418, 513, 532]
posesVect[37] = [294, 847, 279, 744, 462, 561, 358, 666, 507, 516, 354, 726, 248, 775, 671, 409, 507, 526]

'''
#fmr

posesVect[38] = [303, 856, 279, 744, 462, 561, 358, 666, 496, 505, 372, 715, 262, 775, 675, 398, 496, 505]
posesVect[39] = [302, 855, 279, 744, 462, 561, 358, 666, 490, 504, 373, 709, 262, 771, 677, 395, 491, 500]
posesVect[40] = [298, 851, 279, 744, 462, 561, 358, 666, 482, 510, 361, 702, 245, 766, 682, 394, 486, 485]
posesVect[41] = [290, 843, 279, 744, 462, 561, 358, 666, 473, 519, 341, 696, 222, 761, 685, 379, 482, 459]
posesVect[42] = [280, 833, 279, 744, 462, 561, 358, 666, 467, 527, 319, 691, 203, 757, 681, 377, 480, 458]
posesVect[43] = [268, 821, 279, 744, 462, 561, 358, 666, 465, 530, 299, 688, 196, 756, 669, 375, 479, 457]
posesVect[44] = [254, 807, 279, 744, 462, 561, 358, 666, 467, 527, 287, 686, 203, 757, 649, 377, 480, 458]
posesVect[45] = [239, 792, 279, 744, 462, 561, 358, 666, 473, 519, 283, 685, 222, 761, 627, 377, 482, 459]
posesVect[46] = [223, 776, 279, 744, 462, 561, 358, 666, 482, 510, 286, 684, 245, 766, 607, 376, 486, 485]
posesVect[47] = [209, 762, 279, 744, 462, 561, 358, 666, 490, 504, 291, 683, 262, 771, 595, 369, 491, 500]
posesVect[48] = [195, 748, 279, 744, 462, 561, 358, 666, 496, 505, 293, 680, 262, 775, 596, 363, 496, 505]
posesVect[49] = [184, 737, 279, 744, 462, 561, 358, 666, 501, 510, 293, 676, 254, 777, 605, 357, 501, 510]
posesVect[50] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]

#fml

posesVect[51] = [169, 722, 279, 744, 462, 561, 358, 666, 513, 522, 302, 660, 246, 769, 621, 348, 513, 532]
posesVect[52] = [166, 719, 279, 744, 462, 561, 358, 666, 518, 527, 308, 651, 248, 761, 625, 348, 518, 537]
posesVect[53] = [167, 720, 279, 744, 462, 561, 358, 666, 519, 533, 314, 650, 252, 761, 628, 346, 523, 542]
posesVect[54] = [171, 724, 279, 744, 462, 561, 358, 666, 513, 541, 321, 662, 257, 778, 629, 341, 535, 547]
posesVect[55] = [179, 732, 279, 744, 462, 561, 358, 666, 504, 550, 327, 682, 262, 801, 644, 338, 561, 551]
posesVect[56] = [189, 742, 279, 744, 462, 561, 358, 666, 496, 556, 332, 704, 266, 820, 646, 342, 562, 553]
posesVect[57] = [201, 754, 279, 744, 462, 561, 358, 666, 493, 558, 335, 724, 267, 827, 648, 354, 563, 554]
posesVect[58] = [215, 768, 279, 744, 462, 561, 358, 666, 496, 556, 337, 736, 266, 820, 646, 374, 562, 553]
posesVect[59] = [230, 783, 279, 744, 462, 561, 358, 666, 504, 550, 338, 740, 262, 801, 646, 396, 561, 551]
posesVect[60] = [246, 799, 279, 744, 462, 561, 358, 666, 513, 541, 339, 737, 257, 778, 647, 416, 535, 547]
posesVect[61] = [260, 813, 279, 744, 462, 561, 358, 666, 519, 533, 340, 732, 252, 761, 654, 428, 523, 542]
posesVect[62] = [274, 827, 279, 744, 462, 561, 358, 666, 518, 527, 343, 730, 248, 761, 660, 427, 518, 537]
posesVect[63] = [285, 838, 279, 744, 462, 561, 358, 666, 513, 522, 347, 730, 246, 769, 666, 418, 513, 532]
posesVect[64] = [294, 847, 279, 744, 462, 561, 358, 666, 507, 516, 354, 726, 248, 775, 671, 409, 507, 526]

#fmr

posesVect[65] = [303, 856, 279, 744, 462, 561, 358, 666, 496, 505, 372, 715, 262, 775, 675, 398, 496, 505]
posesVect[66] = [302, 855, 279, 744, 462, 561, 358, 666, 490, 504, 373, 709, 262, 771, 677, 395, 491, 500]
posesVect[67] = [298, 851, 279, 744, 462, 561, 358, 666, 482, 510, 361, 702, 245, 766, 682, 394, 486, 485]
posesVect[68] = [290, 843, 279, 744, 462, 561, 358, 666, 473, 519, 341, 696, 222, 761, 685, 379, 482, 459]
posesVect[69] = [280, 833, 279, 744, 462, 561, 358, 666, 467, 527, 319, 691, 203, 757, 681, 377, 480, 458]
posesVect[70] = [268, 821, 279, 744, 462, 561, 358, 666, 465, 530, 299, 688, 196, 756, 669, 375, 479, 457]
posesVect[71] = [254, 807, 279, 744, 462, 561, 358, 666, 467, 527, 287, 686, 203, 757, 649, 377, 480, 458]
posesVect[72] = [239, 792, 279, 744, 462, 561, 358, 666, 473, 519, 283, 685, 222, 761, 627, 377, 482, 459]
posesVect[73] = [223, 776, 279, 744, 462, 561, 358, 666, 482, 510, 286, 684, 245, 766, 607, 376, 486, 485]
posesVect[74] = [209, 762, 279, 744, 462, 561, 358, 666, 490, 504, 291, 683, 262, 771, 595, 369, 491, 500]
posesVect[75] = [195, 748, 279, 744, 462, 561, 358, 666, 496, 505, 293, 680, 262, 775, 596, 363, 496, 505]
posesVect[76] = [184, 737, 279, 744, 462, 561, 358, 666, 501, 510, 293, 676, 254, 777, 605, 357, 501, 510]
posesVect[77] = [175, 728, 279, 744, 462, 561, 358, 666, 507, 516, 297, 669, 248, 775, 614, 352, 507, 516]
'''

def getBalieroJointValue(joint_key):
    if BalieroLucyMapping[key] not in mirrorPitch:
        return poseVectItem[BalieroLucyMapping[key]-1]
    else:
        return 1023 - poseVectItem[BalieroLucyMapping[key]-1]
print "program started"

for i in range(len(posesVect)):
    frame = ET.SubElement(lucyPersistence,  "frame")
    frame.set("number", str(i))
    poseVectItem = posesVect[i]
    for key in robotConfig.getJointsName():
        xmlJoint = ET.SubElement(frame,  key)
        joint_id = configuration.loadJointId(key)
        angle.setValue(getBalieroJointValue(key))
        xmlJointAngle = xmlJoint.set("angle" ,  str(angle.toDegrees()))
        newPose.setValue(key,  angle.toVrep()) #hay que pasarlo a vrep value
    lucy.executePose(newPose)
lucy.stopLucy()
tree = ET.ElementTree(root)
tree.write("baliero.xml")






