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

#include "StdTools/XmlTree.h" 
#include "MultiFingersObjImpController.h"

using namespace MathLib;

MultiFingersObjImpController::MultiFingersObjImpController()
{
    return;
}

MultiFingersObjImpController::~MultiFingersObjImpController()
{
    return;
}

void MultiFingersObjImpController::SetObjCurrentState(const ObjState& state)
{
    mObjectState = state;
    return;
}

void MultiFingersObjImpController::GetObjCurrentState(ObjState& state)
{
    state = mObjectState;
    return;
}

void MultiFingersObjImpController::GetKinematicChain(int ind, KinematicChain* pChain)
{
    if( ind < mFingerDescriptors.size() )
    {
        pChain = &(mFingerDescriptors[ind].mKinChain);
    }
    else
    {
        //printf("Requiring %dth kinematic chain which is invalid...\n", ind);
        pChain = NULL;
    }
    return;
}

void MultiFingersObjImpController::GetDesiredFingerImpedance(int ind, ImpedanceParms& desiredImp)
{
    if(ind < mFingerDescriptors.size())
    {
        desiredImp = mFingerDescriptors[ind].mDesiredImpParms;
    }

    return;
}
void MultiFingersObjImpController::SetDesiredFingerImpedance(int ind, const ImpedanceParms& desiredImp)
{
    if(ind < mFingerDescriptors.size())
    {
        mFingerDescriptors[ind].mDesiredImpParms = desiredImp;
    }

    return;
}

void MultiFingersObjImpController::SetCorrectionStiffness(int ind, double contPnt)
{
	  if(ind < mFingerDescriptors.size())
	    {
	        mFingerDescriptors[ind].CorrectiveStiffness = contPnt;
	    }

	    return;
}

void MultiFingersObjImpController::SetCorrectionPosition(int ind, const Vector3& contPnt)
{
	  if(ind < mFingerDescriptors.size())
	    {
	        mFingerDescriptors[ind].DesiredCorrection = contPnt;
	    }

	    return;
}



void MultiFingersObjImpController::SetContactStiffRestRatio(int ind, double ratio)
{
    if(ind < mFingerDescriptors.size())
    {
        mFingerDescriptors[ind].dContactStiffRestRatio = ratio;
    }
    return;
}

void MultiFingersObjImpController::SetRobotState(const Vector& state)
{
    //write to sensors and update robot link as well as kinematic chains
    mSensorsGroup.SetJointAngles(state);
    mSensorsGroup.WriteSensors();
    mRobot.UpdateLinks();
    for(int ind = 0; ind < mFingerDescriptors.size(); ++ind)
    {
        mFingerDescriptors[ind].mKinChain.Update();
    }

    //update base link reference frame
    mBaseLinkFrame = mRobot.GetReferenceFrame(nBaseLinkID);

    return;
}

void MultiFingersObjImpController::SetFingerChainState(int ind, const Vector& state)
{
    if(ind < mFingerDescriptors.size())
    {
        //map kinematic chain to robot dof
        IndicesVector robotJntVec = mFingerDescriptors[ind].mKinChain.GetJointMapping();

        //trunk the length
        int nLength = state.Size() > robotJntVec.size() ? robotJntVec.size() : state.Size();
        //get current state
        mSensorsGroup.ReadSensors();
        Vector currState = mSensorsGroup.GetJointAngles();
        for(int i = 0; i < nLength; ++i)
        {
            currState(robotJntVec[i]) = state(i);
        }

        //update with new state
        SetRobotState(currState);
    }
    return;
}

void MultiFingersObjImpController::SetContactPoint(int ind, const Vector3& contPnt)
{
    if(ind < mFingerDescriptors.size())
    {
        mFingerDescriptors[ind].mContactPoint = contPnt;
    }
    return;
}

void MultiFingersObjImpController::GetContactPoint(int ind, Vector3& contPnt)
{
    if(ind < mFingerDescriptors.size())
    {
        contPnt = mFingerDescriptors[ind].mContactPoint;
    }
    return;
}

void MultiFingersObjImpController::SetContactForce(int ind, const Vector3& force)
{
    if(ind < mFingerDescriptors.size())
    {
        mFingerDescriptors[ind].mContactForce = force;
    }
    return;
}

void MultiFingersObjImpController::GetContactForce(int ind, Vector3& force)
{
    if(ind < mFingerDescriptors.size())
    {
        force = mFingerDescriptors[ind].mContactForce;
    }
    return;
}

void MultiFingersObjImpController::UpdateFingerContactInfo(int ind, const MathLib::Vector3& force, const MathLib::Vector3& pnt)
{
    //do nothing, the update behavior depends on concrete tactile sensors, inherited class should override this if it would like to use this feature
    return;
}

void MultiFingersObjImpController::GetFKRefFrame(int ind, ReferenceFrame& refFrame)
{
    if(ind < mFingerDescriptors.size())
    {
        mSensorsGroup.ReadSensors();
        Vector currJntPos = mSensorsGroup.GetJointAngles();
        refFrame.Set(mRobot.GetReferenceFrame(mFingerDescriptors[ind].nEndID, mFingerDescriptors[ind].nStartID));
        //printf("Calculate Foward Kinematics between link %d and %d\n", mFingerDescriptors[ind].nStartID, mFingerDescriptors[ind].nEndID);
        //refFrame.GetOrigin().Print();
        //refFrame.GetOrient().Print();
    }

    //printf("Getting FK ref frame done.\n");
    return;
}

void MultiFingersObjImpController::InitRobotStructure(const char* xmlFileName)
{
    //load robotic structure
    LoadStructureXml(xmlFileName);
    //set sensor groups
    mSensorsGroup.SetSensorsList(mRobot.GetSensors());
    //extract kinematic chains
    InitKinematicChain();
    return;
}

void MultiFingersObjImpController::LoadStructureXml(const char* xmlFileName)
{
    XmlTree config;
    bool bSuccess = config.LoadFromFile(xmlFileName);
    if(bSuccess)
    {
        //parse the xml file - config file for RobotToolKit
        pXmlTreeList pTList = config.GetSubTrees();
        vector<string> nameAndPatches;

        FileFinder::AddAdditionalPath("./config");
        FileFinder::AddAdditionalPath("./data");

        for(int i = 0; i < int(pTList->size()); ++i)
        {
            if(pTList->at(i)->GetName() == "Robot")
            {
                printf("Loading robot structure...\n");
                nameAndPatches = Tokenize(RemoveSpaces(pTList->at(i)->GetData()));
                if(nameAndPatches.size() > 0)
                {
                    bSuccess = mRobot.Load(nameAndPatches[0], Serialize(nameAndPatches, 1));
                    if(!bSuccess)
                    {
                        printf("Fail to load robot structure file.\n");
                    }
                    else
                    {
                        printf("Successfully load robot structure.\n");
                    }
                }
            }
        }
    }
    else
    {
        printf("Fail to load configuration file.\n");
    }
    return;
}


