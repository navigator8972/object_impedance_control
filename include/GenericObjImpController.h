/*
 * Copyright (C) 2013 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Hang Yin
 * email:   hang.yin@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/*==================================================================================================
 *
 *  A generic impedance controller for object manipulation
 *
 *================================================================================================*/


#ifndef GenericObjImpController_H_
#define GenericObjImpController_H_

#include "MathLib/MathLib.h"

class ObjState
{
public:
    MathLib::ReferenceFrame     mObjPose;
    MathLib::Vector3            mObjLinVel;
    MathLib::Vector3            mObjAngVel;
    MathLib::Vector3            mObjLinAcc;
    MathLib::Vector3            mObjAngAcc;
};

class ImpedanceParms
{
public:
    //impedance parms for translation
    MathLib::Matrix3        mMassMatrixTrans;
    MathLib::Matrix3        mStiffMatrixTrans;
    MathLib::Matrix3        mDampingMatrixTrans;

    //impedance parms for rotation
    MathLib::Matrix3        mMassMatrixRot;
    MathLib::Matrix3        mStiffMatrixRot;
    MathLib::Matrix3        mDampingMatrixRot;
    // Should we set the option to use quternion?

};

class GenericObjImpController
{
public:
            GenericObjImpController(){};
    virtual ~GenericObjImpController(){};

    virtual void SetDesiredObjState(const ObjState& state){ mDesiredObjState = state; };
    virtual void SetDesiredObjImp(const ImpedanceParms& objImpParm){ mDesiredObjImpedance = objImpParm; };
    virtual void GetDesiredObjState(ObjState& state){ state = mDesiredObjState; };
    virtual void GetDesiredObjImp(ImpedanceParms& objImpParm){ objImpParm = mDesiredObjImpedance; };

    virtual void Update() = 0;
    virtual void GetControlOutput(MathLib::Vector& output) = 0;

protected:
    ObjState        mDesiredObjState;
    ImpedanceParms  mDesiredObjImpedance;
};



#endif 
