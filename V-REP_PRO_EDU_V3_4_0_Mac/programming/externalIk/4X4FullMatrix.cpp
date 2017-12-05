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

#include "MyMath.h"
#include "4X4FullMatrix.h"



C4X4FullMatrix::C4X4FullMatrix()
{
}

C4X4FullMatrix::C4X4FullMatrix(const C4X4Matrix& m)
{
    (*this)=m;
}

C4X4FullMatrix::C4X4FullMatrix(const C4X4FullMatrix& m)
{
    (*this)=m;
}
 
C4X4FullMatrix::~C4X4FullMatrix()
{
} 

void C4X4FullMatrix::clear()
{
    data[0][0]=0.0;
    data[1][0]=0.0;
    data[2][0]=0.0;
    data[3][0]=0.0;
    data[0][1]=0.0;
    data[1][1]=0.0;
    data[2][1]=0.0;
    data[3][1]=0.0;
    data[0][2]=0.0;
    data[1][2]=0.0;
    data[2][2]=0.0;
    data[3][2]=0.0;
    data[0][3]=0.0;
    data[1][3]=0.0;
    data[2][3]=0.0;
    data[3][3]=0.0;
}

void C4X4FullMatrix::setIdentity()
{
    data[0][0]=1.0;
    data[1][0]=0.0;
    data[2][0]=0.0;
    data[3][0]=0.0;
    data[0][1]=0.0;
    data[1][1]=1.0;
    data[2][1]=0.0;
    data[3][1]=0.0;
    data[0][2]=0.0;
    data[1][2]=0.0;
    data[2][2]=1.0;
    data[3][2]=0.0;
    data[0][3]=0.0;
    data[1][3]=0.0;
    data[2][3]=0.0;
    data[3][3]=1.0;
}

C4X4FullMatrix C4X4FullMatrix::operator* (const C4X4FullMatrix& m) const
{
    C4X4FullMatrix retM;
    retM(0,0)=data[0][0]*m(0,0)+data[0][1]*m(1,0)+data[0][2]*m(2,0)+data[0][3]*m(3,0);
    retM(1,0)=data[1][0]*m(0,0)+data[1][1]*m(1,0)+data[1][2]*m(2,0)+data[1][3]*m(3,0);
    retM(2,0)=data[2][0]*m(0,0)+data[2][1]*m(1,0)+data[2][2]*m(2,0)+data[2][3]*m(3,0);
    retM(3,0)=data[3][0]*m(0,0)+data[3][1]*m(1,0)+data[3][2]*m(2,0)+data[3][3]*m(3,0);

    retM(0,1)=data[0][0]*m(0,1)+data[0][1]*m(1,1)+data[0][2]*m(2,1)+data[0][3]*m(3,1);
    retM(1,1)=data[1][0]*m(0,1)+data[1][1]*m(1,1)+data[1][2]*m(2,1)+data[1][3]*m(3,1);
    retM(2,1)=data[2][0]*m(0,1)+data[2][1]*m(1,1)+data[2][2]*m(2,1)+data[2][3]*m(3,1);
    retM(3,1)=data[3][0]*m(0,1)+data[3][1]*m(1,1)+data[3][2]*m(2,1)+data[3][3]*m(3,1);

    retM(0,2)=data[0][0]*m(0,2)+data[0][1]*m(1,2)+data[0][2]*m(2,2)+data[0][3]*m(3,2);
    retM(1,2)=data[1][0]*m(0,2)+data[1][1]*m(1,2)+data[1][2]*m(2,2)+data[1][3]*m(3,2);
    retM(2,2)=data[2][0]*m(0,2)+data[2][1]*m(1,2)+data[2][2]*m(2,2)+data[2][3]*m(3,2);
    retM(3,2)=data[3][0]*m(0,2)+data[3][1]*m(1,2)+data[3][2]*m(2,2)+data[3][3]*m(3,2);

    retM(0,3)=data[0][0]*m(0,3)+data[0][1]*m(1,3)+data[0][2]*m(2,3)+data[0][3]*m(3,3);
    retM(1,3)=data[1][0]*m(0,3)+data[1][1]*m(1,3)+data[1][2]*m(2,3)+data[1][3]*m(3,3);
    retM(2,3)=data[2][0]*m(0,3)+data[2][1]*m(1,3)+data[2][2]*m(2,3)+data[2][3]*m(3,3);
    retM(3,3)=data[3][0]*m(0,3)+data[3][1]*m(1,3)+data[3][2]*m(2,3)+data[3][3]*m(3,3);

    return(retM);
}

C4X4FullMatrix C4X4FullMatrix::operator+ (const C4X4FullMatrix& m) const
{
    C4X4FullMatrix retM;
    retM(0,0)=data[0][0]+m(0,0);
    retM(1,0)=data[1][0]+m(1,0);
    retM(2,0)=data[2][0]+m(2,0);
    retM(3,0)=data[3][0]+m(3,0);
    retM(0,1)=data[0][1]+m(0,1);
    retM(1,1)=data[1][1]+m(1,1);
    retM(2,1)=data[2][1]+m(2,1);
    retM(3,1)=data[3][1]+m(3,1);
    retM(0,2)=data[0][2]+m(0,2);
    retM(1,2)=data[1][2]+m(1,2);
    retM(2,2)=data[2][2]+m(2,2);
    retM(3,2)=data[3][2]+m(3,2);
    retM(0,3)=data[0][3]+m(0,3);
    retM(1,3)=data[1][3]+m(1,3);
    retM(2,3)=data[2][3]+m(2,3);
    retM(3,3)=data[3][3]+m(3,3);
    return(retM);
}

C4X4FullMatrix C4X4FullMatrix::operator- (const C4X4FullMatrix& m) const
{
    C4X4FullMatrix retM;
    retM(0,0)=data[0][0]-m(0,0);
    retM(1,0)=data[1][0]-m(1,0);
    retM(2,0)=data[2][0]-m(2,0);
    retM(3,0)=data[3][0]-m(3,0);
    retM(0,1)=data[0][1]-m(0,1);
    retM(1,1)=data[1][1]-m(1,1);
    retM(2,1)=data[2][1]-m(2,1);
    retM(3,1)=data[3][1]-m(3,1);
    retM(0,2)=data[0][2]-m(0,2);
    retM(1,2)=data[1][2]-m(1,2);
    retM(2,2)=data[2][2]-m(2,2);
    retM(3,2)=data[3][2]-m(3,2);
    retM(0,3)=data[0][3]-m(0,3);
    retM(1,3)=data[1][3]-m(1,3);
    retM(2,3)=data[2][3]-m(2,3);
    retM(3,3)=data[3][3]-m(3,3);
    return(retM);
}

C4X4FullMatrix C4X4FullMatrix::operator* (extIkReal d) const
{
    C4X4FullMatrix retM;
    retM(0,0)=data[0][0]*d;
    retM(1,0)=data[1][0]*d;
    retM(2,0)=data[2][0]*d;
    retM(3,0)=data[3][0]*d;
    retM(0,1)=data[0][1]*d;
    retM(1,1)=data[1][1]*d;
    retM(2,1)=data[2][1]*d;
    retM(3,1)=data[3][1]*d;
    retM(0,2)=data[0][2]*d;
    retM(1,2)=data[1][2]*d;
    retM(2,2)=data[2][2]*d;
    retM(3,2)=data[3][2]*d;
    retM(0,3)=data[0][3]*d;
    retM(1,3)=data[1][3]*d;
    retM(2,3)=data[2][3]*d;
    retM(3,3)=data[3][3]*d;
    return(retM);
}

C4X4FullMatrix C4X4FullMatrix::operator/ (extIkReal d) const
{
    C4X4FullMatrix retM;
    retM(0,0)=data[0][0]/d;
    retM(1,0)=data[1][0]/d;
    retM(2,0)=data[2][0]/d;
    retM(3,0)=data[3][0]/d;
    retM(0,1)=data[0][1]/d;
    retM(1,1)=data[1][1]/d;
    retM(2,1)=data[2][1]/d;
    retM(3,1)=data[3][1]/d;
    retM(0,2)=data[0][2]/d;
    retM(1,2)=data[1][2]/d;
    retM(2,2)=data[2][2]/d;
    retM(3,2)=data[3][2]/d;
    retM(0,3)=data[0][3]/d;
    retM(1,3)=data[1][3]/d;
    retM(2,3)=data[2][3]/d;
    retM(3,3)=data[3][3]/d;
    return(retM);
}

void C4X4FullMatrix::operator*= (const C4X4FullMatrix& m)
{
    (*this)=(*this)*m;
}

void C4X4FullMatrix::operator+= (const C4X4FullMatrix& m)
{
    data[0][0]+=m(0,0);
    data[1][0]+=m(1,0);
    data[2][0]+=m(2,0);
    data[3][0]+=m(3,0);
    data[0][1]+=m(0,1);
    data[1][1]+=m(1,1);
    data[2][1]+=m(2,1);
    data[3][1]+=m(3,1);
    data[0][2]+=m(0,2);
    data[1][2]+=m(1,2);
    data[2][2]+=m(2,2);
    data[3][2]+=m(3,2);
    data[0][3]+=m(0,3);
    data[1][3]+=m(1,3);
    data[2][3]+=m(2,3);
    data[3][3]+=m(3,3);
}

void C4X4FullMatrix::operator-= (const C4X4FullMatrix& m)
{
    data[0][0]-=m(0,0);
    data[1][0]-=m(1,0);
    data[2][0]-=m(2,0);
    data[3][0]-=m(3,0);
    data[0][1]-=m(0,1);
    data[1][1]-=m(1,1);
    data[2][1]-=m(2,1);
    data[3][1]-=m(3,1);
    data[0][2]-=m(0,2);
    data[1][2]-=m(1,2);
    data[2][2]-=m(2,2);
    data[3][2]-=m(3,2);
    data[0][3]-=m(0,3);
    data[1][3]-=m(1,3);
    data[2][3]-=m(2,3);
    data[3][3]-=m(3,3);
}

void C4X4FullMatrix::operator*= (extIkReal d)
{
    data[0][0]*=d;
    data[1][0]*=d;
    data[2][0]*=d;
    data[3][0]*=d;
    data[0][1]*=d;
    data[1][1]*=d;
    data[2][1]*=d;
    data[3][1]*=d;
    data[0][2]*=d;
    data[1][2]*=d;
    data[2][2]*=d;
    data[3][2]*=d;
    data[0][3]*=d;
    data[1][3]*=d;
    data[2][3]*=d;
    data[3][3]*=d;
}

void C4X4FullMatrix::operator/= (extIkReal d)
{
    data[0][0]/=d;
    data[1][0]/=d;
    data[2][0]/=d;
    data[3][0]/=d;
    data[0][1]/=d;
    data[1][1]/=d;
    data[2][1]/=d;
    data[3][1]/=d;
    data[0][2]/=d;
    data[1][2]/=d;
    data[2][2]/=d;
    data[3][2]/=d;
    data[0][3]/=d;
    data[1][3]/=d;
    data[2][3]/=d;
    data[3][3]/=d;
}

C4X4FullMatrix& C4X4FullMatrix::operator= (const C4X4Matrix& m)
{
    data[0][0]=m.M.axis[0](0);
    data[1][0]=m.M.axis[0](1);
    data[2][0]=m.M.axis[0](2);
    data[3][0]=0.0;

    data[0][1]=m.M.axis[1](0);
    data[1][1]=m.M.axis[1](1);
    data[2][1]=m.M.axis[1](2);
    data[3][1]=0.0;

    data[0][2]=m.M.axis[2](0);
    data[1][2]=m.M.axis[2](1);
    data[2][2]=m.M.axis[2](2);
    data[3][2]=0.0;

    data[0][3]=m.X(0);
    data[1][3]=m.X(1);
    data[2][3]=m.X(2);
    data[3][3]=1.0;

    return(*this);
}

C4X4FullMatrix& C4X4FullMatrix::operator= (const C4X4FullMatrix& m)
{
    data[0][0]=m(0,0);
    data[1][0]=m(1,0);
    data[2][0]=m(2,0);
    data[3][0]=m(3,0);
    data[0][1]=m(0,1);
    data[1][1]=m(1,1);
    data[2][1]=m(2,1);
    data[3][1]=m(3,1);
    data[0][2]=m(0,2);
    data[1][2]=m(1,2);
    data[2][2]=m(2,2);
    data[3][2]=m(3,2);
    data[0][3]=m(0,3);
    data[1][3]=m(1,3);
    data[2][3]=m(2,3);
    data[3][3]=m(3,3);
    return(*this);
}

void C4X4FullMatrix::invert()
{ // Write a faster routine later!  
    C4X4FullMatrix n;

    n(0,0)=data[0][0];
    n(1,0)=data[0][1];
    n(2,0)=data[0][2];
    n(3,0)=data[0][3];

    n(0,1)=data[1][0];
    n(1,1)=data[1][1];
    n(2,1)=data[1][2];
    n(3,1)=data[1][3];

    n(0,2)=data[2][0];
    n(1,2)=data[2][1];
    n(2,2)=data[2][2];
    n(3,2)=data[2][3];

    n(0,3)=data[3][0];
    n(1,3)=data[3][1];
    n(2,3)=data[3][2];
    n(3,3)=data[3][3];

    (*this)=n;
}

void C4X4FullMatrix::buildZRotation(extIkReal angle)
{
    extIkReal c=(extIkReal)cos(angle);
    extIkReal s=(extIkReal)sin(angle);
    data[0][0]=c;
    data[0][1]=-s;
    data[0][2]=0.0;
    data[0][3]=0.0;
    data[1][0]=s;
    data[1][1]=c;
    data[1][2]=0.0;
    data[1][3]=0.0;
    data[2][0]=0.0;
    data[2][1]=0.0;
    data[2][2]=1.0;
    data[2][3]=0.0;
    data[3][0]=0.0;
    data[3][1]=0.0;
    data[3][2]=0.0;
    data[3][3]=1.0;
}

void C4X4FullMatrix::buildTranslation(extIkReal x, extIkReal y, extIkReal z)
{
    data[0][0]=1.0;
    data[0][1]=0.0;
    data[0][2]=0.0;
    data[0][3]=x;
    data[1][0]=0.0;
    data[1][1]=1.0;
    data[1][2]=0.0;
    data[1][3]=y;
    data[2][0]=0.0;
    data[2][1]=0.0;
    data[2][2]=1.0;
    data[2][3]=z;
    data[3][0]=0.0;
    data[3][1]=0.0;
    data[3][2]=0.0;
    data[3][3]=1.0;
}

C3Vector C4X4FullMatrix::getEulerAngles() const
{ // Angles are in radians!! // THERE IS ANOTHER SUCH ROUTINE IN C4X4MATRIX
    C3Vector retV;
    extIkReal m02=data[0][2];
    if (m02>1.0)
        m02=1.0;    // Just in case
    if (m02<-1.0)
        m02=-1.0;   // Just in case

    retV(1)=CMath::robustAsin(m02);
    if (m02<0.0)
        m02=-m02;
    if (m02<0.999995)
    {   // No gimbal lock
        retV(0)=(extIkReal)atan2(-data[1][2],data[2][2]);
        retV(2)=(extIkReal)atan2(-data[0][1],data[0][0]);
    }
    else
    {   // Gimbal lock has occured
        retV(0)=0.0;
        retV(2)=(extIkReal)atan2(data[1][0],data[1][1]);
    }
    return(retV);
}
