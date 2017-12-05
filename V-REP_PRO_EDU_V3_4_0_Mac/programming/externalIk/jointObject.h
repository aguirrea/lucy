// This file is part of the EXTERNAL IK for V-REP
// 
// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The EXTERNAL IK is licensed under the terms of EITHER:
//   1. EXTERNAL IK commercial license (contact us for details)
//   2. EXTERNAL IK educational license (see below)
// 
// EXTERNAL IK educational license:
// -------------------------------------------------------------------
// The EXTERNAL IK educational license applies only to EDUCATIONAL
// ENTITIES composed by following people and institutions:
// 
// 1. Hobbyists, students, teachers and professors
// 2. Schools and universities
// 
// EDUCATIONAL ENTITIES do NOT include companies, research institutions,
// non-profit organisations, foundations, etc.
// 
// An EDUCATIONAL ENTITY may use, modify, compile and distribute the
// modified/unmodified EXTERNAL IK under following conditions:
//  
// 1. Distribution should be free of charge.
// 2. Distribution should be to EDUCATIONAL ENTITIES only.
// 3. Usage should be non-commercial.
// 4. Altered source versions must be plainly marked as such and distributed
//    along with any compiled code.
// 5. The origin of the EXTERNAL IK must not be misrepresented. you must
//    not claim that you wrote the original software.
// 
// THE EXTERNAL IK IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

#pragma once

#include "extIk.h"
#include "3DObject.h"

class CJoint : public C3DObject  
{
public:
    CJoint(); // default, use only during serialization!
    CJoint(int jointType);
    virtual ~CJoint();

public:
    void commonInit();

    bool announceObjectWillBeErased(int objID,bool copyBuffer);
    void announceIkObjectWillBeErased(int ikGroupID,bool copyBuffer);

    void performObjectLoadingMapping(std::vector<int>* map);

    extIkReal getPosition(bool useTempValues=false);
    void setPosition(extIkReal parameter,bool useTempValues=false);
    extIkReal getPosition_ratio();
    void setPosition_ratio(extIkReal posRatio);

    void initializeParametersForIK(extIkReal angularJointLimitationThreshold);
    int getDoFs();
    void getLocalTransformationExPart1(C7Vector& mTr,int index,bool useTempValues=false);
    extIkReal getTempParameterEx(int index);
    void setTempParameterEx(extIkReal parameter,int index);
    void applyTempParametersEx();
    int getTempSphericalJointLimitations();

    extIkReal getScrewPitch() const;
    void setSphericalTransformation(const C4Vector& tr);
    C4Vector getSphericalTransformation() const;
    int getJointType();
    extIkReal getPositionIntervalMin();
    void setPositionIntervalMin(extIkReal m);
    extIkReal getPositionIntervalRange();
    void setPositionIntervalRange(extIkReal r);

    bool getPositionIsCyclic();
    void setPositionIsCyclic(bool c);

    extIkReal getIKWeight();
    void setIKWeight(extIkReal newWeight);
    void setMaxStepSize(extIkReal stepS);
    extIkReal getMaxStepSize(); 

    void _rectifyDependentJoints();

    void setJointMode(int theMode);
    int getJointMode();
    int getDependencyJointID();
    extIkReal getDependencyJointCoeff();
    extIkReal getDependencyJointFact();

    // Various
    std::vector<CJoint*> directDependentJoints;

protected:
    // Variables which need to be serialized & copied
    // Main joint attributes:
    int _jointType;
    C4Vector _sphericalTransformation;
    bool _positionIsCyclic;
    extIkReal _screwPitch;
    extIkReal _jointMinPosition;
    extIkReal _jointPositionRange;

    // Joint state and other:
    extIkReal _jointPosition;

    // IK and path planning calculation:
    extIkReal _maxStepSize;

    // General IK calculation:
    extIkReal _ikWeight;

    int _jointMode;
    int _dependencyJointID;
    extIkReal _dependencyJointCoeff;
    extIkReal _dependencyJointFact;

    // Temporary values used when doing IK:
    extIkReal _jointPosition_tempForIK;
    extIkReal _sphericalTransformation_euler1TempForIK;
    extIkReal _sphericalTransformation_euler2TempForIK;
    extIkReal _sphericalTransformation_euler3TempForIK;
    int _sphericalTransformation_eulerLockTempForIK; // bit-coded, bit0--> _sphericalTransformation_euler1TempForIK, bit1--> _sphericalTransformation_euler2TempForIK, etc.

public:
    void serializeRExtIk(CExtIkSer& ar);
};
