// This file is part of the COLLADA PLUGIN for V-REP
// 
// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The COLLADA PLUGIN is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The COLLADA PLUGIN is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// THE COLLADA PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
//  
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the COLLADA PLUGIN.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

// Written by Alex Doumanoglou on behalf of Dr. Marc Freese

#pragma once

class Transformation
{
public:
    enum TransformationType
    {
        Matrix,
        Translate,
        Rotate,
        Scale
    };

private:

    TransformationType m_TransformationType;

public:

    virtual ~Transformation() { };
    TransformationType getTransformationType() const { return m_TransformationType; }
    virtual Transformation* clone() const = 0;          // use delete for desctruction

protected:

    Transformation(TransformationType Type)
    {
        m_TransformationType = Type;
    }

private:
    // Disable default assignment operator.
    const Transformation& operator=(const Transformation& trans) { return *this; };

};
