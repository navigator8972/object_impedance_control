/*
 * Copyright (C) 2013 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Hang Yin
 * email:   hang.yin@.epfl.ch
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

/*===========================================================================================
 *
 *  An implementation of generic object impedance controller for multiple fingers robotic hand
 *
 * ==========================================================================================*/

#ifndef MultiFingersObjImpController_H_
#define MultiFingersObjImpController_H_

#include <vector>
#include "RobotLib/KinematicChain.h"
#include "GenericObjImpController.h"

class FingerDescriptor
{
public:
    int                 nStartID;
    int                 nEndID;
    KinematicChain      mKinChain;
    ImpedanceParms      mDesiredImpParms;           //<hyin/Jul-17-2013> note that the specified impedance is described in local frame (finger tip)
    Vector3             mContactPoint;
    Vector3             mContactForce;
    double              dContactStiffRestRatio;     //<hyin/Jul-17-2013> it's better to be specified as a ratio thus user needs not to be care about concrete rest length

    Vector3             DesiredCorrection;         //Miao: add the grasp correction part (absolute value);
    double              CorrectiveStiffness;       //Miao: The corrective stiffness which move the finger to desired position;

};

class MultiFingersObjImpController : public GenericObjImpController
{
public:
            MultiFingersObjImpController();
    virtual ~MultiFingersObjImpController();

    virtual void    SetObjCurrentState(const ObjState& state);              //if obj state could be sensed...
    virtual void    GetObjCurrentState(ObjState& state);
    virtual void    GetKinematicChain(int ind, KinematicChain* pChain);
    virtual void    SetRobotState(const MathLib::Vector& state);
    virtual void    SetFingerChainState(int ind, const MathLib::Vector& state);
    virtual void    GetDesiredFingerImpedance(int ind, ImpedanceParms& desiredImp);
    virtual void    SetDesiredFingerImpedance(int ind, const ImpedanceParms& desiredImp);
    virtual void    SetContactStiffRestRatio(int ind, double ratio);
    virtual void    SetContactPoint(int ind, const Vector3& contPnt);

    virtual void    SetCorrectionStiffness(int ind, double contPnt);             //Miao
    virtual void    SetCorrectionPosition(int ind, const Vector3& contPnt);      //Miao

    virtual void    GetContactPoint(int ind, Vector3& contPnt);
    virtual void    SetContactForce(int ind, const Vector3& force);
    virtual void    GetContactForce(int ind, Vector3& force);
    virtual void    GetFKRefFrame(int ind, ReferenceFrame& refFrame);
    virtual void    GetBaseRefFrame(ReferenceFrame& refFrame){ refFrame.Set(mBaseLinkFrame); };
    virtual void    InitRobotStructure(const char* xmlFileName);
    virtual void    UpdateFingerContactInfo(int ind, const MathLib::Vector3& force, const MathLib::Vector3& pnt);
protected:
    Robot                                           mRobot;
    RevoluteJointSensorGroup                        mSensorsGroup;
    ObjState                                        mObjectState;
    std::vector< FingerDescriptor >                 mFingerDescriptors;
    int                                             nBaseLinkID;
    ReferenceFrame                                  mBaseLinkFrame;         //the hand frame with respect to base link of whole chain (might include arm)

    virtual void    LoadStructureXml(const char* xmlFileName);
    virtual void    InitKinematicChain() = 0;
    virtual void    GetJacobian(MathLib::Matrix& jacobian) = 0;             //get a combined jacobian
    virtual void    GetJacobian(int ind, MathLib::Matrix& jacobian) = 0;    //get jacobian with respect to specified chain
    virtual void    GetGMatrix(MathLib::Matrix& gMatrix) = 0;
    virtual void    GetGMatrix(int ind, MathLib::Matrix& gMatrix) = 0;      //get G matrix with respect to specified chain
};



#endif 
