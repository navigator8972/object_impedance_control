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

/*===========================================================================
 *
 *  An implementation on Allegro Hand
 *
 * =========================================================================*/

#ifndef AllegroHandObjImpController_H_
#define AllegroHandObjImpController_H_

#include "MultiFingersObjImpController.h"

//#define USE_GMM_WORKSPACE_REACTION
#define USE_SYNTOUCH_FEEDBACK
#ifdef USE_GMM_WORKSPACE_REACTION
#include "Gaussians.h"
#endif

class AllegroHandObjImpController : public MultiFingersObjImpController
{
public:
            AllegroHandObjImpController();
            AllegroHandObjImpController(const char* config);
    virtual ~AllegroHandObjImpController();

    virtual void GetObjCurrentState(ObjState& state);
    virtual void GetObjCurrentStateNoTac(ObjState& state); // Miao: get the VF from the finger kinematics, no inforamtion from tactile sensing. T

    virtual void Update();
    virtual void GetControlOutput(MathLib::Vector& output);

#ifdef USE_GMM_WORKSPACE_REACTION
    virtual void InitGMMWorkspaceDesc(int nDataDim, int nState, const char* prior_file, const char* mu_file, const char* sigma_file);
    virtual void ReleaseGMMWorkspaceDesc();
    virtual void GetWorkspaceReactionOutput(MathLib::Vector& output);
#endif

#ifdef USE_SYNTOUCH_FEEDBACK
    //function to update contact information for specified finger
    //note that given contact point might not be described in the same base frame, one need to override this to translate it
    //to frames interests
    virtual void UpdateFingerContactInfo(int ind, const MathLib::Vector3& force, const MathLib::Vector3& pnt);
    //get finger contact point frame described in hand frame
    virtual void GetFingerContactPntFrm(int ind, MathLib::ReferenceFrame& contactFrm);
#endif

protected:

    virtual void InitKinematicChain();                              //init kinematic chain, assign proper model to each chain
    virtual void GetJacobian(MathLib::Matrix& jacobian);
    virtual void GetJacobian(int ind, MathLib::Matrix& jacobian);
    virtual void GetGMatrix(MathLib::Matrix& gMatrix);
    virtual void GetGMatrix(int ind, MathLib::Matrix& gMatrix);
    virtual void GetAttitudeControl(std::vector< MathLib::Vector >& control);

    virtual void UpdateGraspControl();
    virtual void UpdateObjPoseControl();

    virtual void UpdateCorrectionControl();  // Miao: add the correction part for each finger;

#ifdef USE_SYNTOUCH_FEEDBACK
    //function to calculate torque generated from offset of contact point from center of finger tip sphere
    //and convert to joint space through jacobian
    virtual void GetControlFromContactOffset(int ind, const Vector& force, Vector& control);
    MathLib::ReferenceFrame mSynTouchOffFrm;
#endif

    ObjState            mCurrObjState;
    MathLib::Vector     mGraspControl;
    MathLib::Vector     mObjMotionControl;
    MathLib::Vector     mCorrectionControl;

#ifdef USE_GMM_WORKSPACE_REACTION
    double              dInnerThreshold;
    double              dOutThreshold;
    MathLib::Vector     mWorkspaceReactionControl;
    Gaussians*          pWorkspaceReactionGMM;
#endif
};



#endif 
