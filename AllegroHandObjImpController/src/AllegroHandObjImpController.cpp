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

#include "AllegroHandObjImpController.h"

using namespace MathLib;

AllegroHandObjImpController::AllegroHandObjImpController()
:MultiFingersObjImpController(){

#ifdef USE_GMM_WORKSPACE_REACTION
	pWorkspaceReactionGMM = NULL;
#endif
}
AllegroHandObjImpController::~AllegroHandObjImpController()
{
#ifdef USE_GMM_WORKSPACE_REACTION
	ReleaseGMMWorkspaceDesc();
#endif
}

AllegroHandObjImpController::AllegroHandObjImpController(const char* configFileName)
:MultiFingersObjImpController()
{
	InitRobotStructure(configFileName);

	//by default the desired object stiffness is identity * scale
	mDesiredObjImpedance.mStiffMatrixTrans.Identity();
	/*
    mDesiredObjImpedance.mStiffMatrixTrans(0, 0) = 20;
    mDesiredObjImpedance.mStiffMatrixTrans(1, 1) = 240;
    mDesiredObjImpedance.mStiffMatrixTrans(2, 2) = 20;
	 */
	mDesiredObjImpedance.mStiffMatrixTrans = mDesiredObjImpedance.mStiffMatrixTrans * 80;
	mDesiredObjImpedance.mStiffMatrixRot.Identity();
	/*
    mDesiredObjImpedance.mStiffMatrixRot(0, 0) = 10;
    mDesiredObjImpedance.mStiffMatrixRot(1, 1) = 50;
    mDesiredObjImpedance.mStiffMatrixRot(2, 2) = 10;
	 */
	mDesiredObjImpedance.mStiffMatrixRot = mDesiredObjImpedance.mStiffMatrixRot * 2;
	//three fingers involved;
	mObjMotionControl.Resize(12);
	mGraspControl.Resize(12);
	mCorrectionControl.Resize(12);
	mObjMotionControl.Zero();
	mGraspControl.Zero();
	mCorrectionControl.Zero();

#ifdef USE_GMM_WORKSPACE_REACTION
	mWorkspaceReactionControl.Resize(12);
	pWorkspaceReactionGMM = NULL;
	//dInnerThreshold means workspace reaction should be activated since the object is approaching the boundary of workspace
	dInnerThreshold = 7.7;
	//dOutThreashold means workspace reaction should be even stronger since current state is intolerable...
	dOutThreshold = 5.3;
#endif

#ifdef USE_SYNTOUCH_FEEDBACK
	Vector3 relOrigin;
	Matrix3 relOrient;

	relOrigin(0) = -4.75 + 7.0 * 0.3420 /*sin20*/;
	relOrigin(1) = 0.0;
	relOrigin(2) = 25.0 + 7.0 * 0.9397 /*cos20*/;
	relOrigin *= 0.001;                 //mm to m
	relOrient(0, 0) = 0.0; relOrient(0, 1) = 0.3420 /*sin20*/; relOrient(0, 2) = 0.9397 /*cos20*/;
	relOrient(1, 0) = 1.0; relOrient(1, 1) = 0.0; relOrient(1, 2) = 0.0;
	relOrient(2, 0) = 0.0; relOrient(2, 1) = 0.9397 /*cos20*/; relOrient(2, 2) = -0.3420 /*sin-20*/;
	mSynTouchOffFrm.SetOrigin(relOrigin);
	mSynTouchOffFrm.SetOrient(relOrient);
#endif

	return;
}

void AllegroHandObjImpController::GetObjCurrentState(ObjState& state)
{   // here is the place that define the virtual frame;

	//printf("Getting obj virtual frame.\n");
	//override default object state, here it is actually from virtual frame...
	//check sum of force and reference frame of end link of each finger
	double sumForceNorm = 0.0;
	ReferenceFrame refFrameList[3];
	for(int i = 0; i < mFingerDescriptors.size(); ++i)
	{
#ifdef USE_SYNTOUCH_FEEDBACK
		//<hyin/Aug-23rd-2013> need to update the fk frame according to syntouch reading, origin should be contact point while the axes remain unchanged - note that theoretically it's better to also update it according to norm, but currently just one norm direction is known, cross product with existing axis is possible but better to have tangential direction...
		GetFingerContactPntFrm(i, refFrameList[i]);
		//should we also update the weight according to empirical contact force reading?
		//sumForceNorm += mFingerDescriptors[i].mContactForce.Norm();
#else
		GetFKRefFrame(i, refFrameList[i]);
#endif
	}
	//printf("Getting virtual frame origin.\n");
	Vector3 virtFrameOrigin;
	virtFrameOrigin.Zero();
	if(sumForceNorm < 1e-4)
	{
		//now, actually the object is not grasped, but assum the virtual frame is as the case of equal contact forces
		for(int i = 0; i < mFingerDescriptors.size(); ++i)
		{
			Vector3 tmpVec(refFrameList[i].GetOrigin());
			//tmpVec.Print();
			virtFrameOrigin += tmpVec;
		}
		virtFrameOrigin = virtFrameOrigin / mFingerDescriptors.size();
	}
	else
	{
		//calculate virtual frame origin through weighted combination
		for(int i = 0; i < mFingerDescriptors.size(); ++i)
		{
			Vector3 tmpVec(refFrameList[i].GetOrigin());
			virtFrameOrigin += tmpVec * mFingerDescriptors[i].mContactForce.Norm();
		}
		//normalize it...
		virtFrameOrigin = virtFrameOrigin / sumForceNorm;
	}

	//now construct rotation matrix for virtual frame
	//printf("construct rotation matrix for virtual frame.\n");
	Vector3 rx, ry, rz;
	Vector3 x31(refFrameList[2].GetOrigin() - refFrameList[0].GetOrigin());
	//normalize it
	rx = x31 / x31.Norm();

	Vector3 x21(refFrameList[1].GetOrigin() - refFrameList[0].GetOrigin());
	Vector3 x21rx = x21.Cross(rx);
	rz = x21rx / x21rx.Norm();

	ry = rz.Cross(rx);

	//fill obj state structure
	Matrix3 virtFrameOrient;
	virtFrameOrient.SetColumns(rx, ry, rz);
	state.mObjPose.SetOrigin(virtFrameOrigin);
	state.mObjPose.SetOrient(virtFrameOrient);
	//virtFrameOrigin.Print();
	//virtFrameOrient.Print();
	//
	return;
}

//----Miao, this part is for kenji's experiments and get the VF without tactile information. The original definition is above.

void AllegroHandObjImpController::GetObjCurrentStateNoTac(ObjState& state)
{   // here is the place that define the virtual frame;

	//printf("Getting obj virtual frame.\n");
	//override default object state, here it is actually from virtual frame...
	//check sum of force and reference frame of end link of each finger
	double sumForceNorm = 0.0;
	ReferenceFrame refFrameList[3];
	for(int i = 0; i < mFingerDescriptors.size(); ++i)
	{
		GetFKRefFrame(i, refFrameList[i]);
	}
	//printf("Getting virtual frame origin.\n");
	Vector3 virtFrameOrigin;
	virtFrameOrigin.Zero();
	if(sumForceNorm < 1e-4)
	{
		//now, actually the object is not grasped, but assume the virtual frame is as the case of equal contact forces
		for(int i = 0; i < mFingerDescriptors.size(); ++i)
		{
			Vector3 tmpVec(refFrameList[i].GetOrigin());
			//tmpVec.Print();
			virtFrameOrigin += tmpVec;
		}
		virtFrameOrigin = virtFrameOrigin / mFingerDescriptors.size();
	}
	else
	{
		//calculate virtual frame origin through weighted combination
		for(int i = 0; i < mFingerDescriptors.size(); ++i)
		{
			Vector3 tmpVec(refFrameList[i].GetOrigin());
			virtFrameOrigin += tmpVec * mFingerDescriptors[i].mContactForce.Norm();
		}
		//normalize it...
		virtFrameOrigin = virtFrameOrigin / sumForceNorm;
	}

	//now construct rotation matrix for virtual frame
	//printf("construct rotation matrix for virtual frame.\n");
	Vector3 rx, ry, rz;
	Vector3 x31(refFrameList[2].GetOrigin() - refFrameList[0].GetOrigin());
	//normalize it
	rx = x31 / x31.Norm();

	Vector3 x21(refFrameList[1].GetOrigin() - refFrameList[0].GetOrigin());
	Vector3 x21rx = x21.Cross(rx);
	rz = x21rx / x21rx.Norm();

	ry = rz.Cross(rx);

	//fill obj state structure
	Matrix3 virtFrameOrient;
	virtFrameOrient.SetColumns(rx, ry, rz);
	state.mObjPose.SetOrigin(virtFrameOrigin);
	state.mObjPose.SetOrient(virtFrameOrient);
	//virtFrameOrigin.Print();
	//virtFrameOrient.Print();
	//
	return;
}
//---Miao


void AllegroHandObjImpController::InitKinematicChain()
{
	//assign proper kinematic chain
	FingerDescriptor indexFinger;
	FingerDescriptor middleFinger;
	FingerDescriptor thumbFinger;

	mFingerDescriptors.push_back(indexFinger);      /*0*/
	mFingerDescriptors.push_back(middleFinger);     /*1*/
	mFingerDescriptors.push_back(thumbFinger);      /*2*/

	mFingerDescriptors[0].mKinChain.SetRobot(&mRobot);
	mFingerDescriptors[1].mKinChain.SetRobot(&mRobot);
	mFingerDescriptors[2].mKinChain.SetRobot(&mRobot);


	int nBaseID = mRobot.GetLinkIndex("TOOL");
	int nIndexBaseID = mRobot.GetLinkIndex("link_0_0");
	int nMiddleBaseID = mRobot.GetLinkIndex("link_4_0");
	int nThumbBaseID = mRobot.GetLinkIndex("link_12_0");


	printf("nBaseId: %d, nIndexId: %d, nMiddleBaseId: %d, nThumbBaseId: %d\n", nBaseID, nIndexBaseID, nMiddleBaseID, nThumbBaseID);

	mFingerDescriptors[0].nStartID = nBaseID;
	mFingerDescriptors[0].nEndID = mRobot.GetLinkIndex("link_3_0_tip");
	mFingerDescriptors[0].mKinChain.Create(nBaseID, nBaseID, mFingerDescriptors[0].nEndID);
	mFingerDescriptors[0].mDesiredImpParms.mStiffMatrixTrans.Zero();    //by default the desired stiffness for finger is 50, this might not be a proper value, need further investigation
	mFingerDescriptors[0].mDesiredImpParms.mStiffMatrixTrans(0, 0) = 20;
	mFingerDescriptors[0].mDesiredImpParms.mStiffMatrixTrans(1, 1) = 5;
	mFingerDescriptors[0].mDesiredImpParms.mStiffMatrixTrans(2, 2) = 5;
	//Miao
	mFingerDescriptors[0].CorrectiveStiffness=0;



	//mFingerDescriptors[0].mDesiredImpParms.mStiffMatrixTrans *= 80;
	mFingerDescriptors[0].dContactStiffRestRatio = -2;      //assume the rest length is 80% of virtual link between finger and obj

	mFingerDescriptors[1].nStartID = nBaseID;
	mFingerDescriptors[1].nEndID = mRobot.GetLinkIndex("link_7_0_tip");
	mFingerDescriptors[1].mKinChain.Create(nBaseID, nBaseID, mFingerDescriptors[1].nEndID);
	mFingerDescriptors[1].mDesiredImpParms.mStiffMatrixTrans.Zero();
	mFingerDescriptors[1].mDesiredImpParms.mStiffMatrixTrans(0, 0) = 20;
	mFingerDescriptors[1].mDesiredImpParms.mStiffMatrixTrans(1, 1) = 5;
	mFingerDescriptors[1].mDesiredImpParms.mStiffMatrixTrans(2, 2) = 5;
	//mFingerDescriptors[1].mDesiredImpParms.mStiffMatrixTrans *= 80;
	mFingerDescriptors[1].dContactStiffRestRatio = -2;

	mFingerDescriptors[1].CorrectiveStiffness=0;

	mFingerDescriptors[2].nStartID = nBaseID;
	mFingerDescriptors[2].nEndID = mRobot.GetLinkIndex("link_15_0_tip");
	mFingerDescriptors[2].mKinChain.Create(nBaseID, nBaseID, mFingerDescriptors[2].nEndID);
	mFingerDescriptors[2].mDesiredImpParms.mStiffMatrixTrans.Zero();
	mFingerDescriptors[2].mDesiredImpParms.mStiffMatrixTrans(0, 0) = 20;
	mFingerDescriptors[2].mDesiredImpParms.mStiffMatrixTrans(1, 1) = 5;
	mFingerDescriptors[2].mDesiredImpParms.mStiffMatrixTrans(2, 2) = 5;

	//mFingerDescriptors[2].mDesiredImpParms.mStiffMatrixTrans *= 80;
	mFingerDescriptors[2].dContactStiffRestRatio = -2;

	mFingerDescriptors[2].CorrectiveStiffness=0;

	//initialize base frame with respect to world
	nBaseLinkID = nBaseID;
	mBaseLinkFrame.Set(mRobot.GetReferenceFrame(nBaseID));

	return;
}

void AllegroHandObjImpController::Update()
{
	//core stuff should be done here...
	//update current object state
	GetObjCurrentState(mCurrObjState);
	//printf("Updating grasp control...\n");
	UpdateGraspControl();
	//printf("Updating pose control...\n");
	UpdateObjPoseControl();

	UpdateCorrectionControl();  //Miao: 2013-Oct-25

#ifdef USE_GMM_WORKSPACE_REACTION
	GetWorkspaceReactionOutput(mWorkspaceReactionControl);
#endif
	return;
}

//Miao---
void AllegroHandObjImpController::UpdateCorrectionControl()
{
	Vector3 desiredIndexOrigin, desiredMiddleOrigin, desiredThumbOrigin;
	ReferenceFrame currPose[3];

	GetFKRefFrame(0, currPose[0]);
	GetFKRefFrame(1, currPose[1]);
	GetFKRefFrame(2, currPose[2]);

	Vector3 currIndexOrigin(currPose[0].GetOrigin());
	Vector3 currMiddleOrigin(currPose[1].GetOrigin());
	Vector3 currThumbOrigin(currPose[2].GetOrigin());

	Vector3 deltaX[3];
	deltaX[0] = mFingerDescriptors[0].DesiredCorrection - currIndexOrigin;
	deltaX[1] = mFingerDescriptors[1].DesiredCorrection - currMiddleOrigin;
	deltaX[2] = mFingerDescriptors[2].DesiredCorrection - currThumbOrigin;

	Matrix jacobianIndex, jacobianMiddle, jacobianThumb;
	GetJacobian(0, jacobianIndex);
	GetJacobian(1, jacobianMiddle);
	GetJacobian(2, jacobianThumb);

	Vector indexTrq(4), middleTrq(4), thumbTrq(4);

	Vector deltaXvec[3];

	deltaXvec[0].Set(deltaX[0].Array(),3);
	deltaXvec[1].Set(deltaX[1].Array(),3);
    deltaXvec[2].Set(deltaX[2].Array(),3);

	indexTrq = jacobianIndex.GetRows(0, 2).TransposeMult(deltaXvec[0])*mFingerDescriptors[0].CorrectiveStiffness;

	middleTrq = jacobianMiddle.GetRows(0, 2).TransposeMult(deltaXvec[1])*mFingerDescriptors[1].CorrectiveStiffness;

	thumbTrq = jacobianThumb.GetRows(0, 2).TransposeMult(deltaXvec[2])*mFingerDescriptors[2].CorrectiveStiffness;

	mCorrectionControl.SetSubVector(0, indexTrq);
	mCorrectionControl.SetSubVector(4, middleTrq);
	mCorrectionControl.SetSubVector(8, thumbTrq);

		return;

}
//Miao---

void AllegroHandObjImpController::UpdateGraspControl()
{
	//do not need to consider torque at contact point
	//<hyin/Jul-26-2013> note for grasp the desired obj origin is current virtual frame origin, not overall desired obj pose
	Vector3 desiredObjOrigin(mCurrObjState.mObjPose.GetOrigin());

	//desired position for each finger tip
	//note that in Kenji's paper, the desired origin depends on whether contact forces are specified...
	Vector3 desiredIndexOrigin, desiredMiddleOrigin, desiredThumbOrigin;
	ReferenceFrame currPose[3];

	GetFKRefFrame(0, currPose[0]);
	GetFKRefFrame(1, currPose[1]);
	GetFKRefFrame(2, currPose[2]);

	Vector3 currIndexOrigin(currPose[0].GetOrigin());
	Vector3 currMiddleOrigin(currPose[1].GetOrigin());
	Vector3 currThumbOrigin(currPose[2].GetOrigin());

	//printf("desired obj origin:\n");
	//desiredObjOrigin.Print();

	Vector3 deltaX[3];

	deltaX[0] = desiredObjOrigin - currIndexOrigin;     //index
	//printf("current index tip origin:\n");
	//currIndexOrigin.Print();
	//printf("delta vector of index:\n");
	//deltaX[0].Print();

	deltaX[1] = desiredObjOrigin - currMiddleOrigin;    //middle
	//printf("current middle tip origin:\n");
	//currMiddleOrigin.Print();
	//printf("delta vector of middle:\n");
	//deltaX[1].Print();

	deltaX[2] = desiredObjOrigin - currThumbOrigin;     //thumb
	//printf("current thumb tip origin:\n");
	//currThumbOrigin.Print();
	//printf("delta vector of thumb:\n");
	//deltaX[2].Print();
	//printf("Norm of delta thumb: %d\n", deltaX[2].Norm());
	//substract rest length
	//printf("Taking rest length into account...\n");
	Vector3 deltaXIndex = deltaX[0]  * (1 - mFingerDescriptors[0].dContactStiffRestRatio);
	Vector3 deltaXMiddle = deltaX[1] * (1 - mFingerDescriptors[1].dContactStiffRestRatio);
	Vector3 deltaXThumb = deltaX[2]  * (1 - mFingerDescriptors[2].dContactStiffRestRatio);

	//calculate desired torque from specified stiffness matrix
	//prepare jacobian as well as g matrix
	Matrix jacobianIndex, jacobianMiddle, jacobianThumb;
	GetJacobian(0, jacobianIndex);
	GetJacobian(1, jacobianMiddle);
	GetJacobian(2, jacobianThumb);
	//printf("Index jacobian:\n");
	//jacobianIndex.Print();
	//printf("Thumb jacobian:\n");
	//jacobianThumb.Print();

	Vector indexTrq(4), middleTrq(4), thumbTrq(4);
	//desired stiffness matrix, need to be transformed to global frame
	//to construct transform matrix (from local contact point to global frame)
	//x: vector between obj and contact point
	//y: cross of x and z of finger tip (tangential with finger longitudinal direction)
	//z: cross of x and y
	//<hyin/Jul-22-2013> this seems not to be supported theoretically, imagine what will happen
	//if the two directions coincide. Changed to this
	//x: vector between obj and contact point
	//y: cross of x and virtual linkage direction between another finger (circular relation, index -> middle, middle -> thumb, thumb -> index
	//z: cross of x and y
	//Also, it is possible to ignore other direction, then directly represent desired principle stiffness to in local frame
	Matrix3 tipToGlobal[3];
	for(int i = 0; i < 3; ++i)
	{
		//<hyin/Jul-24-2013> note that cross product of unit vectors is not necessarily unit vector! need to normalize it...
		Vector3 tmpXAxis = deltaX[i] / deltaX[i].Norm();
		Vector3 tmpYAxis = tmpXAxis.Cross(currPose[i].GetOrient().GetColumn(2));
		tmpYAxis = tmpYAxis / tmpYAxis.Norm();
		/*
        Vector3 virtLinkAxis = currPose[(i + 1) % 3].GetOrigin() - currPose[i].GetOrigin();
        Vector3 tmpYAxis = tmpXAxis.Cross(virtLinkAxis);
        tmpYAxis = tmpYAxis / tmpYAxis.Norm();
		 */
		Vector3 tmpZAxis = tmpXAxis.Cross(tmpYAxis);
		tmpZAxis = tmpZAxis / tmpZAxis.Norm();
		tipToGlobal[i].SetColumns(tmpXAxis, tmpYAxis, tmpZAxis);
	}
	Matrix3 desiredIndexStiff, desiredMiddleStiff, desiredThumbStiff;
	Matrix3 desiredIndexStiffTmp, desiredMiddleStiffTmp, desiredThumbStiffTmp;
	//printf("tipToGlobal[0]:\n");
	//tipToGlobal[0].Print();
	//desired_World = R_T /dot desired_Local /dot R
	//<hyin/Jul-23-2013> note the above formula is not correct: should follow
	//desired_World = R /dot desired_Local /dot R_T
	//no let's first keep it
	tipToGlobal[0].Mult(mFingerDescriptors[0].mDesiredImpParms.mStiffMatrixTrans, desiredIndexStiffTmp);
	tipToGlobal[1].Mult(mFingerDescriptors[1].mDesiredImpParms.mStiffMatrixTrans, desiredMiddleStiffTmp);
	tipToGlobal[2].Mult(mFingerDescriptors[2].mDesiredImpParms.mStiffMatrixTrans, desiredThumbStiffTmp);

	desiredIndexStiffTmp.Mult(tipToGlobal[0].Transpose(), desiredIndexStiff);
	desiredMiddleStiffTmp.Mult(tipToGlobal[1].Transpose(), desiredMiddleStiff);
	desiredThumbStiffTmp.Mult(tipToGlobal[2].Transpose(), desiredThumbStiff);

	/*
    //printf("desired index stiff matrix in global reference frame.\n");
    //desiredIndexStiff.Print();
    //printf("desired thumb stiff matrix in global reference frame.\n");
    //desiredThumbStiff.Print();

    //\tau = J_T \dot K \dot \delta x 
    Matrix jntStiffMatIndex, jntStiffMatMiddle, jntStiffMatThumb;
    jacobianIndex.GetRows(0, 2).TransposeMult(desiredIndexStiff, jntStiffMatIndex);
    jacobianMiddle.GetRows(0, 2).TransposeMult(desiredMiddleStiff, jntStiffMatMiddle);
    jacobianThumb.GetRows(0, 2).TransposeMult(desiredThumbStiff, jntStiffMatThumb);


    //check joint stiffness
    //jntStiffMatIndex.Print();
    //jntStiffMatMiddle.Print();
    //jntStiffMatThumb.Print();

    indexTrq = jntStiffMatIndex.Mult(deltaXIndex);    //only position component needed
    middleTrq = jntStiffMatMiddle.Mult(deltaXMiddle);
    thumbTrq = jntStiffMatThumb.Mult(deltaXThumb);
	 */
	Vector graspResForceVec[3];
	Matrix desiredIndexStiffMat(desiredIndexStiff), desiredMiddleStiffMat(desiredMiddleStiff), desiredThumbStiffMat(desiredThumbStiff);

	desiredIndexStiffMat.Mult(deltaXIndex, graspResForceVec[0]);
	desiredMiddleStiffMat.Mult(deltaXMiddle, graspResForceVec[1]);
	desiredThumbStiffMat.Mult(deltaXThumb, graspResForceVec[2]);
	indexTrq = jacobianIndex.GetRows(0, 2).TransposeMult(graspResForceVec[0]);
	middleTrq = jacobianMiddle.GetRows(0, 2).TransposeMult(graspResForceVec[1]);
	thumbTrq = jacobianThumb.GetRows(0, 2).TransposeMult(graspResForceVec[2]);

#ifdef USE_SYNTOUCH_FEEDBACK
	//we can no longer assum the force is applied on center of finger tip sphere but also consider the torque effect
	Vector trqFromOffset[3];
	for(int i = 0; i < 3; ++i)
	{
		GetControlFromContactOffset(i, graspResForceVec[i], trqFromOffset[i]);
	}
	indexTrq = indexTrq + trqFromOffset[0];
	middleTrq = middleTrq + trqFromOffset[1];
	thumbTrq = thumbTrq + trqFromOffset[2];
#endif

	//update grasp control output
	mGraspControl.SetSubVector(0, indexTrq);
	mGraspControl.SetSubVector(4, middleTrq);
	mGraspControl.SetSubVector(8, thumbTrq);

	return;
}

void AllegroHandObjImpController::UpdateObjPoseControl()
{
	//consider translation first
	Vector3 desiredOrigin(mDesiredObjState.mObjPose.GetOrigin());
	Matrix3 desiredOrient(mDesiredObjState.mObjPose.GetOrient());

	Vector3 currVirtOrigin(mCurrObjState.mObjPose.GetOrigin());
	Matrix3 currVirtOrient(mCurrObjState.mObjPose.GetOrient());

	//translational error
	Vector transErr(desiredOrigin - currVirtOrigin);
	//printf("Translational error:\n");
	//transErr.Print();

	//prepare jacobian as well as g matrix
	Matrix jacobianIndex, jacobianMiddle, jacobianThumb;
	Matrix gMatrixIndex, gMatrixMiddle, gMatrixThumb;
	GetJacobian(0, jacobianIndex);
	GetJacobian(1, jacobianMiddle);
	GetJacobian(2, jacobianThumb);

	GetGMatrix(0, gMatrixIndex);
	GetGMatrix(1, gMatrixMiddle);
	GetGMatrix(2, gMatrixThumb);

	//desired stiffness matrix
	const Matrix& desiredTransStiff = mDesiredObjImpedance.mStiffMatrixTrans;
	//const Matrix& desiredRotStiff = mDesiredObjImpedance.mStiffMatrixRot;

	Vector indexTransTrq(4), middleTransTrq(4), thumbTransTrq(4);
	vector< Vector > attitudeTrq(3);

	/*
    //extract translational as well as rotational parts
    Matrix jntTransStiffMatIndex, jntTransStiffMatMiddle, jntTransStiffMatThumb;
    //Matrix jntRotStiffMatIndex, jntRotStiffMatMiddle, jntRotStiffMatThumb;
    //translational

    jacobianIndex.GetRows(0, 2).TransposeMult(gMatrixIndex, jntTransStiffMatIndex);
    jntTransStiffMatIndex = jntTransStiffMatIndex.Mult(desiredTransStiff);
    jacobianMiddle.GetRows(0, 2).TransposeMult(gMatrixMiddle, jntTransStiffMatMiddle);
    jntTransStiffMatMiddle = jntTransStiffMatMiddle.Mult(desiredTransStiff);
    jacobianThumb.GetRows(0, 2).TransposeMult(gMatrixThumb, jntTransStiffMatThumb);
    jntTransStiffMatThumb = jntTransStiffMatThumb.Mult(desiredTransStiff);

    indexTransTrq = jntTransStiffMatIndex.Mult(transErr);
    middleTransTrq = jntTransStiffMatMiddle.Mult(transErr);
    thumbTransTrq = jntTransStiffMatThumb.Mult(transErr);
	 */
	//<hyin/Aug-23rd-2013> reformulate above calculation to first calculate restoring force in cartesian space and then convert
	//to joint space. This will be convenient for calculating torque from offset of contact point with respect to center
	//of finger tip sphere
	Matrix cartTransStiffMat[3];
	Vector cartResForceVec[3];
	gMatrixIndex.Mult(desiredTransStiff, cartTransStiffMat[0]);
	gMatrixMiddle.Mult(desiredTransStiff, cartTransStiffMat[1]);
	gMatrixThumb.Mult(desiredTransStiff, cartTransStiffMat[2]);
	for(int i = 0; i < 3; ++i)
	{
		cartTransStiffMat[i].Mult(transErr, cartResForceVec[i]);
	}
	indexTransTrq = jacobianIndex.GetRows(0, 2).TransposeMult(cartResForceVec[0]);
	middleTransTrq = jacobianMiddle.GetRows(0, 2).TransposeMult(cartResForceVec[1]);
	thumbTransTrq = jacobianThumb.GetRows(0, 2).TransposeMult(cartResForceVec[2]);

#ifdef USE_SYNTOUCH_FEEDBACK
	//we can no longer assum the force is applied on center of finger tip sphere but also consider the torque effect
	Vector trqFromOffset[3];
	for(int i = 0; i < 3; ++i)
	{
		GetControlFromContactOffset(i, cartResForceVec[i], trqFromOffset[i]);
	}
	indexTransTrq = indexTransTrq + trqFromOffset[0];
	middleTransTrq = middleTransTrq + trqFromOffset[1];
	thumbTransTrq = thumbTransTrq + trqFromOffset[2];
#endif

	//rotational: stiffness matrix is specified in global frame, calculate delta_R and then convert it to unit quaternion, apply vector part to stiffness matrix
	//<hyin/Jul-29-2013> try attitude control derived in Kenji's paper
	GetAttitudeControl(attitudeTrq);
	/*
    for(int i = 0; i < attitudeTrq.size(); ++i)
    {
        printf("Atittude control for finger %d\n", i + 1);
        attitudeTrq[i].Print();
    }
	 */
	//update obj motion control output
	mObjMotionControl.SetSubVector(0, indexTransTrq + attitudeTrq[0]);
	mObjMotionControl.SetSubVector(4, middleTransTrq + attitudeTrq[1]);
	mObjMotionControl.SetSubVector(8, thumbTransTrq + attitudeTrq[2]);
	return;
}

void AllegroHandObjImpController::GetAttitudeControl( vector< Vector >& attitudeControl)
{
	//consider rotation
	Vector3 desiredOrigin(mDesiredObjState.mObjPose.GetOrigin());
	Matrix3 desiredOrient(mDesiredObjState.mObjPose.GetOrient());

	Vector3 currVirtOrigin(mCurrObjState.mObjPose.GetOrigin());
	Matrix3 currVirtOrient(mCurrObjState.mObjPose.GetOrient());


	//prepare jacobian as well as g matrix
	//note actually here it is not the real G matrix, it's just a heuristics for distribute force to multiple fingers...
	Matrix jacobian[3];
	Matrix gMatrix[3];
	Matrix3 gMatrix3[3];
	ReferenceFrame currPose[3];
	Vector3 currOrigin[3];

	for(int i = 0; i < 3; ++i)
	{
		GetJacobian(i, jacobian[i]);
		GetGMatrix(i, gMatrix[i]);
		gMatrix3[i].Set(gMatrix[i]);
		GetFKRefFrame(i, currPose[i]);
		currOrigin[i].Set(currPose[i].GetOrigin());
	}

	//desired stiffness matrix
	const Matrix3& desiredRotStiff = mDesiredObjImpedance.mStiffMatrixRot;

	Vector3 omega[3];

	Vector3 rx, ry, rz, rxd, ryd, rzd;
	currVirtOrient.GetColumn(0, rx);
	currVirtOrient.GetColumn(1, ry);
	currVirtOrient.GetColumn(2, rz);
	desiredOrient.GetColumn(0, rxd);
	desiredOrient.GetColumn(1, ryd);
	desiredOrient.GetColumn(2, rzd);

	Vector3 rx_rxd_prod, ry_ryd_prod, rz_rzd_prod;
	rx.Cross(rxd, rx_rxd_prod);
	ry.Cross(ryd, ry_ryd_prod);
	rz.Cross(rzd, rz_rzd_prod);

	Vector3 omega_vec_origin = rx_rxd_prod + ry_ryd_prod + rz_rzd_prod;


	//<hyin/Jul-30-2013> use quaternion to represent attitude error
	Matrix3 deltaRot;
	//currVirtOrient.TransposeMult(desiredOrient, deltaRot);
	//desiredOrient.TransposeMult(currVirtOrient, deltaRot);
	//<hyin/Jul-31-2013> this is contrary to rotational error as we expected, but actually it gives same direction with sum of cross products above...
	//another thing I found is currVirtOrient.TransposeMult(desiredOrient, deltaRot) fits direction of sum of cross products with its eigen vector of eigen value 1
	desiredOrient.Mult(currVirtOrient.Transpose(), deltaRot);
	/*
    printf("deltaRot:\n");
    deltaRot.Print();
	 */

	Vector deltaQuat;
	deltaRot.GetQuaternionRepresentation(deltaQuat);
	Vector3 omega_vec;
	//extract orientation part
	omega_vec(0) = deltaQuat(1);
	omega_vec(1) = deltaQuat(2);
	omega_vec(2) = deltaQuat(3);
	/*
    printf("Original deltaRot:\n");
    omega_vec_origin.Print();
    printf("Revised deltaRot:\n");
    omega_vec.Print();
	 */
	Matrix3 An, At;
	//An = rz * rz' / ||rz||^2
	//At = I - An
	double rzSquare = rz.Norm() * rz.Norm();
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
		{
			An(i, j) = rz(i) * rz(j) / rzSquare;
			if(i == j)
			{
				At(i, j) = 1 - An(i, j);
			}
			else
			{
				At(i, j) = - An(i, j);
			}
		}
	}

	for(int i = 0; i < 3; ++i)
	{
		//for each finger tip...
		attitudeControl[i].Resize(4);
		omega[i] = desiredRotStiff.Mult(omega_vec_origin);
		//omega[i] = gMatrix3[i].Mult(omega[i]);

		//lni = xo - xi
		//lti = lni' * (rz.Cross(At * omega[i])) * (rz.Cross(At * omega[i])) / (At * omega[i]).Norm()^2
		//lti = 0 if (At * omega[i]).Norm() is zero
		Vector3 l_ni(currVirtOrigin - currOrigin[i]);
		Vector3 l_ti;
		Vector3 At_omega = At.Mult(omega[i]);
		Vector3 An_omega = An.Mult(omega[i]);
		if(At_omega.Norm() < 1e-2)
		{
			l_ti.Zero();
		}
		else
		{
			Vector3 rz_cross_At_omega;
			rz.Cross(At_omega, rz_cross_At_omega);
			l_ti = rz_cross_At_omega * l_ni.Dot(rz_cross_At_omega) / (rz_cross_At_omega.Norm() * rz_cross_At_omega.Norm());
		}
		/*
        printf("At * omega:\n");
        At_omega.Print();
        printf("l_ni for finger %d:\n", i);
        l_ni.Print();
        printf("l_ti for finger %d:\n", i);
        l_ti.Print();
		 */

		Vector3 f_ni_vec3, f_ti_vec3;
		l_ni.Cross(An_omega, f_ni_vec3);
		l_ti.Cross(At_omega, f_ti_vec3);
		Vector f_ni(f_ni_vec3);
		Vector f_ti(f_ti_vec3);
		//f_ni = f_ni / (l_ni.Norm() * l_ni.Norm());
		//f_ti = f_ti / (l_ti.Norm() * l_ti.Norm());
		/*
        printf("F_ni for finger %d:\n", i);
        f_ni.Print();
        printf("F_ti for finger %d:\n", i);
        f_ti.Print();
		 */
		//printf("Sum force of f_ni and f_ti for finger %d:\n", i);
		Vector sum_f(f_ni + f_ti);
		//sum_f.Print();
		jacobian[i].GetRows(0, 2).TransposeMult(sum_f, attitudeControl[i]);
#ifdef USE_SYNTOUCH_FEEDBACK
		Vector trqFromOffset;
		GetControlFromContactOffset(i, sum_f, trqFromOffset);
		attitudeControl[i] = attitudeControl[i] + trqFromOffset;
#endif
		/*
        printf("Jacobian for finger %d:\n", i);
        jacobian[i].GetRows(0, 2).Print();
        printf("Attitude control torque for finger %d:\n", i);
        attitudeControl[i].Print();
		 */
	}
	return;
}

void AllegroHandObjImpController::GetControlOutput(Vector& output)
{
	//return computed control signal
	//printf("Obj Motion Component:\n");
	//mObjMotionControl.Print();
	//printf("Grasp Component:\n");
	//mGraspControl.Print();
	output = mObjMotionControl + mGraspControl+mCorrectionControl; //Miao

	mCorrectionControl.Print("Correction Control");

#ifdef USE_GMM_WORKSPACE_REACTION
	output = output + mWorkspaceReactionControl;
	//printf("Reaction Component:\n");
	//mWorkspaceReactionControl.Print();
#endif
	return;
}

void AllegroHandObjImpController::GetJacobian(Matrix& jacobian)
{
	//return combined jacobian
	return;
}

void AllegroHandObjImpController::GetJacobian(int ind, Matrix& jacobian)
{
	//return jacobian for specified finger
	if(ind < mFingerDescriptors.size())
	{
		jacobian = mFingerDescriptors[ind].mKinChain.GetJacobian();
	}
	return;
}

void AllegroHandObjImpController::GetGMatrix(Matrix& gMatrix)
{
	//return combined G matrix
	return;
}

void AllegroHandObjImpController::GetGMatrix(int ind, Matrix& gMatrix)
{
	//return G matrix for specified finger
	gMatrix.Resize(3, 3);
	gMatrix.Identity();
	//for virtual frame, it is just diagonal matrix consists of weight for specified finger
	//X_o = Sigma( f_i * X_i ) / Sigma( f_i )
	double sumForceNorm = 0.0;
#ifdef USE_SYNTOUCH_FEEDBACK
	for(int i = 0; i < mFingerDescriptors.size(); ++i)
	{
		sumForceNorm += mFingerDescriptors[i].mContactForce.Norm();
	}
#endif

	if(sumForceNorm < 1e-4)
	{
		gMatrix(0, 0) = 1.0 / 3;
		gMatrix(1, 1) = 1.0 / 3;
		gMatrix(2, 2) = 1.0 / 3;
	}
	else
	{
		gMatrix(0, 0) = mFingerDescriptors[ind].mContactForce.Norm() / sumForceNorm;
		gMatrix(1, 1) = mFingerDescriptors[ind].mContactForce.Norm() / sumForceNorm;
		gMatrix(2, 2) = mFingerDescriptors[ind].mContactForce.Norm() / sumForceNorm;
	}

	return;
}

#ifdef USE_GMM_WORKSPACE_REACTION
void AllegroHandObjImpController::InitGMMWorkspaceDesc(int nDataDim, int nState, const char* prior_file, const char* mu_file, const char* sigma_file)
{
	if(pWorkspaceReactionGMM != NULL)
	{
		//release current model
		ReleaseGMMWorkspaceDesc();
	}

	pWorkspaceReactionGMM = new Gaussians(nState, nDataDim, mu_file, sigma_file, prior_file);
	if(pWorkspaceReactionGMM != NULL)
	{
		pWorkspaceReactionGMM->InitFastGaussians(0, 2);
		printf("Initialize GMM model...\n");
		printf("Number of Gaussian components: %d; Number of data dimension: %d\n", pWorkspaceReactionGMM->model.nbStates, pWorkspaceReactionGMM->model.nbDim);
		for(int i = 0; i < nState; ++i)
		{
			printf("State %d Prior: %lf\n", i, pWorkspaceReactionGMM->model.States[i].Prio);
			printf("Mu:\n");
			pWorkspaceReactionGMM->model.States[i].Mu.Print();
			printf("Sigma:\n");
			pWorkspaceReactionGMM->model.States[i].Sigma.Print();
		}
	}

	return;
}

void AllegroHandObjImpController::ReleaseGMMWorkspaceDesc()
{
	if(pWorkspaceReactionGMM != NULL)
	{
		delete pWorkspaceReactionGMM;
		pWorkspaceReactionGMM = NULL;
	}
	return;
}

void AllegroHandObjImpController::GetWorkspaceReactionOutput(MathLib::Vector& output)
{
	if(pWorkspaceReactionGMM == NULL)
	{
		output.Zero();
		return;
	}
	//get current virtual frame to see if workspace reaction needs to be activated
	Vector3 desiredOrigin(mDesiredObjState.mObjPose.GetOrigin());
	Matrix3 desiredOrient(mDesiredObjState.mObjPose.GetOrient());

	//note the model is trained with origin within arm global frame, we also need to enquiry with that transformation
	ReferenceFrame baseFrame, objFrameWorld;
	GetBaseRefFrame(baseFrame);
	objFrameWorld.Set(baseFrame.Mult(mCurrObjState.mObjPose));

	Vector3 currVirtOriginArmGlobal(objFrameWorld.GetOrigin());

	Vector3 currVirtOrigin(mCurrObjState.mObjPose.GetOrigin());
	Matrix3 currVirtOrient(mCurrObjState.mObjPose.GetOrient());

	Vector transErr(desiredOrigin - currVirtOrigin);
	//prepare jacobian as well as g matrix
	Matrix jacobianIndex, jacobianMiddle, jacobianThumb;
	Matrix gMatrixIndex, gMatrixMiddle, gMatrixThumb;
	GetJacobian(0, jacobianIndex);
	GetJacobian(1, jacobianMiddle);
	GetJacobian(2, jacobianThumb);

	GetGMatrix(0, gMatrixIndex);
	GetGMatrix(1, gMatrixMiddle);
	GetGMatrix(2, gMatrixThumb);

	//a reaction stiffness;
	Matrix3 reactStiffMatrix;
	reactStiffMatrix.Identity();

	//construct input for model
	Vector modelInput(3);
	modelInput(0) = currVirtOriginArmGlobal(0);
	modelInput(1) = currVirtOriginArmGlobal(1);
	modelInput(2) = currVirtOriginArmGlobal(2);
	/*
    modelInput(3) = currVirtOrient(0, 0);
    modelInput(4) = currVirtOrient(1, 0);
    modelInput(5) = currVirtOrient(2, 0);
    modelInput(6) = currVirtOrient(1, 0);
    modelInput(7) = currVirtOrient(1, 1);
    modelInput(8) = currVirtOrient(1, 2);
	 */

	//printf("Model input:\n");
	//modelInput.Print();
	double prob = pWorkspaceReactionGMM->GaussianProbFast(modelInput);
	//printf("Workspace prob: %lf\n", prob);
	double log_likelihood = log(prob);
	//printf("Workspace log-likelihood: %lf\n", log_likelihood);

	if(log_likelihood > dInnerThreshold)
	{
		//obj is happy with current pose, no reaction is applied
		reactStiffMatrix.Zero();
	}
	else if(log_likelihood > dOutThreshold)
	{
		//apply some reaction to drive obj back to desired pose
		printf("Out of inner region! log-likelihood: %lf\n", log_likelihood);
		reactStiffMatrix = reactStiffMatrix * 50;
		//reactStiffMatrix.Print();

	}
	else
	{
		//it's dangerous state, apply even stronger reaction
		printf("Out of outer region!\n log-likelihood: %lf\n", log_likelihood);
		reactStiffMatrix = reactStiffMatrix * 80;
		//reactStiffMatrix.Print();
	}

	Vector indexTrq(4), middleTrq(4), thumbTrq(4);
	/*
    //extract translational as well as rotational parts
    Matrix jntStiffMatIndex, jntStiffMatMiddle, jntStiffMatThumb;
    //translational
    jacobianIndex.GetRows(0, 2).TransposeMult(gMatrixIndex, jntStiffMatIndex);
    jntStiffMatIndex = jntStiffMatIndex.Mult(reactStiffMatrix);
    jacobianMiddle.GetRows(0, 2).TransposeMult(gMatrixMiddle, jntStiffMatMiddle);
    jntStiffMatMiddle = jntStiffMatMiddle.Mult(reactStiffMatrix);
    jacobianThumb.GetRows(0, 2).TransposeMult(gMatrixThumb, jntStiffMatThumb);
    jntStiffMatThumb = jntStiffMatThumb.Mult(reactStiffMatrix);

    indexTrq = jntStiffMatIndex.Mult(transErr);    //only position component needed
    middleTrq = jntStiffMatMiddle.Mult(transErr);
    thumbTrq = jntStiffMatThumb.Mult(transErr);
	 */
	//<hyin/Aug-23rd-2013> need to add trq generated from offset of contact point with respect to center of finger tip sphere
	Matrix cartTransStiffMat[3];
	Vector cartResForceVec[3];
	gMatrixIndex.Mult(reactStiffMatrix, cartTransStiffMat[0]);
	gMatrixMiddle.Mult(reactStiffMatrix, cartTransStiffMat[1]);
	gMatrixThumb.Mult(reactStiffMatrix, cartTransStiffMat[2]);
	for(int i = 0; i < 3; ++i)
	{
		cartTransStiffMat[i].Mult(transErr, cartResForceVec[i]);
	}
	indexTransTrq = jacobianIndex.GetRows(0, 2).TransposeMult(cartResForceVec[0]);
	middleTransTrq = jacobianMiddle.GetRows(0, 2).TransposeMult(cartResForceVec[1]);
	thumbTransTrq = jacobianThumb.GetRows(0, 2).TransposeMult(cartResForceVec[2]);

#ifdef USE_SYNTOUCH_FEEDBACK
	//we can no longer assume the force is applied on center of finger tip sphere but also consider the torque effect
	Vector trqFromOffset[3];
	for(int i = 0; i < 3; ++i)
	{
		GetControlFromContactOffset(i, cartResForceVec[i], trqFromOffset[i]);
	}
	indexTransTrq = indexTransTrq + trqFromOffset[0];
	middleTransTrq = middleTransTrq + trqFromOffset[1];
	thumbTransTrq = thumbTransTrq + trqFromOffset[2];
#endif


	//update obj motion control output
	mWorkspaceReactionControl.SetSubVector(0, indexTrq);
	mWorkspaceReactionControl.SetSubVector(4, middleTrq);
	mWorkspaceReactionControl.SetSubVector(8, thumbTrq);

	return;
}
#endif

#ifdef USE_SYNTOUCH_FEEDBACK
void AllegroHandObjImpController::UpdateFingerContactInfo(int ind, const Vector3& force, const Vector3& pnt)
{
	//the contact point is defined under the frame of corresponding finger tip. the benefit of this definition is that
	//if something unexpected happened such that pnt is invalid as zero vector, then this is degenerated to the case of center of finger tip sphere...
	if(ind < mFingerDescriptors.size())
	{
		//pnt is described in a frame locates at some point of sensor basement, not finger tip. Need transform here...no, just record it...
		SetContactPoint(ind, pnt);

		//Fill force value
		SetContactForce(ind, force);
	}
	return;
}

void AllegroHandObjImpController::GetFingerContactPntFrm(int ind, MathLib::ReferenceFrame& contactFrm)
{
	if(ind < mFingerDescriptors.size())
	{
		GetFKRefFrame(ind, contactFrm);
		contactFrm = contactFrm.Mult(mSynTouchOffFrm);
		contactFrm.SetOrigin(contactFrm.GetOrigin() + contactFrm.GetOrient().Mult(mFingerDescriptors[ind].mContactPoint));
	}
	return;
}

void AllegroHandObjImpController::GetControlFromContactOffset(int ind, const Vector& force, Vector& control)
{
	//current just return zero for control
	control.Resize(4);
	control.Zero();
	if(force.Size() < 3)
	{
		return;
	}
	Matrix jacobian;
	GetJacobian(ind, jacobian);
	//the torque effect from the force need to be calculated, both force & arm should be represented under global frame
	ReferenceFrame fingertipFrm, contactFrm;
	GetFKRefFrame(ind, fingertipFrm);
	GetFingerContactPntFrm(ind, contactFrm);
	Vector3 relVec(contactFrm.GetOrigin() - fingertipFrm.GetOrigin());
	//relVec.Print();
	Vector3 forceVec;
	forceVec(0) = force(0);
	forceVec(1) = force(1);
	forceVec(2) = force(2);
	Vector3 equTrq = relVec.Cross(forceVec);
	//jacobian.GetRows(3, 5).Print();
	//forceVec.Print();
	jacobian.GetRows(3, 5).TransposeMult(equTrq, control);
//	control.Print();
	//control.Zero();
	return;
}
#endif

extern "C"{
// These two "C" functions manage the creation and destruction of the class
AllegroHandObjImpController* create(){return new AllegroHandObjImpController();}
void destroy(AllegroHandObjImpController* module){delete module;}
}

