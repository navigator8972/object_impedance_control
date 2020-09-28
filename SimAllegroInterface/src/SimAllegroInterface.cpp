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

#include "SimAllegroInterface.h"
#include "GLTools/GLTools.h"
//#include "RobotGUI/SimWorldCmd.h"
//#include "Utilities.h"
#include <iostream>
#include <fstream>

#define SIM_WORLD_CMD                   "/RobotToolKitSimulator/SimWorldCmd"
#define KUKA_CMD_TOPIC                  "/kuka_lwr/joint_cmd"
//#define ALLEGRO_CMD_TOPIC               "/allegro/joint_cmd"
//the simulated robot is just a slave one so the cmd is actually from state of real robot
#define ALLEGRO_CMD_TOPIC               "/allegro/joint_state"
#define OBJ_POSE_TOPIC                  "/kuka_allegro/obj_pose"

#define OBJ_DATA_FILE           "data/Worlds/Examples/Objects/TrajBall.xml"

//natural frequency for CDDynamics
#define WN_JOINT_PLANNER                (1.7)
#define WN_JOINT_FILTER                 (1.7)
#define UPDATE_INTERVAL                 (1.0/500.0)


void GetRotationMatrixFromQuaternion(const Vector& quat, Matrix3& mat)
{
    //vector should be a normalized quaternion
    if(quat.Size() != 4)
    {
        return;
    }
    else
    {
        //see wikipedia - quaternion and spatial rotation
        double a = quat(0), b = quat(1), c = quat(2), d = quat(3);
        //printf("w:%lf, x:%lf, y:%lf, z:%lf\n", a, b, c, d);
        double rot11 = a * a + b * b - c * c - d * d;
        double rot12 = 2 * b * c - 2 * a * d;
        double rot13 = 2 * b * d + 2 * a * c;
        double rot21 = 2 * b * c + 2 * a * d;
        double rot22 = a * a - b * b + c * c - d * d;
        double rot23 = 2 * c * d - 2 * a * b;
        double rot31 = 2 * b * d - 2 * a * c;
        double rot32 = 2 * c * d + 2 * a * b;
        double rot33 = a * a - b * b - c * c + d * d;

        mat.Set(rot11, rot12, rot13,
                rot21, rot22, rot23,
                rot31, rot32, rot33);
    }

    return;
}

//velocity limit for kuka lwr joints
Vector kukaLWRVelLimit;

SimAllegroInterface::SimAllegroInterface()
:RobotInterface(){
    eMotionCmdType = MOTION_CMD_TYPE_POSITION;
    mLWRRobot = NULL;
    nHandBaseId = -1;
    nJointNum = -1;
    bStart = false;
    bTopicReady = false;
    int nStateID = -1;

    pRosNode = NULL;
    pArmCDDyn = NULL;
    pHandCDDyn = NULL;
    bool bArmCDDynReady = false;
    bool bHandCDDynReady = false;
    bool bObjFrameOn = false;

}
SimAllegroInterface::~SimAllegroInterface(){
}

RobotInterface::Status SimAllegroInterface::RobotInit(){

    //initialize robot - this would be a combination of kuka lwr and allegro robot
    mLWRRobot = (LWRRobot*) mRobot;

    if(NULL == mLWRRobot)
    {
        printf("Fail to get LWR & Allegro Hand robot.\n");
        return STATUS_ERROR;
    }
    nStateID = 0;           //idle

    AddConsoleCommand("start");
    AddConsoleCommand("stop");
    AddConsoleCommand("obj_frame");
    AddConsoleCommand("fk_index");
    AddConsoleCommand("fk_middle");
    AddConsoleCommand("fk_thumb");

    mLWRRobot->SetControlMode(Robot::CTRLMODE_POSITION);

    mSensorsGroup.SetSensorsList(mLWRRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mLWRRobot->GetActuators());

    nJointNum = mLWRRobot->GetDOFCount();
    nHandBaseId = mLWRRobot->GetDOFIndex("HandF1Dof0");     //base index of allegro hand link - the name is for right hand...
    if(nHandBaseId < 0)
    {
        //try left hand
        nHandBaseId = mLWRRobot->GetDOFIndex("link_0_0");
        if(nHandBaseId < 0)
        {
            //no hand is associated
            nHandBaseId = nJointNum;
        }
    }

    printf("Successfully load robot structure - Total DOFs: %d; Hand base index: %d\n", nJointNum, nHandBaseId);
    mArmJointTargetPosition.Resize(nHandBaseId);
    mHandJointTargetPosition.Resize(nJointNum - nHandBaseId);
    mArmJointTargetVel.Resize(nHandBaseId);
    mHandJointTargetVel.Resize(nJointNum - nHandBaseId);

    //velocity limit for KUKA DOFs, from Seungsu's catching package
    kukaLWRVelLimit.Resize(nHandBaseId);
    if(kukaLWRVelLimit.Size() > 6)
    {
        kukaLWRVelLimit(0) = DEG2RAD(132.0) * 0.90;
        kukaLWRVelLimit(1) = DEG2RAD(132.0) * 0.90;
        kukaLWRVelLimit(2) = DEG2RAD(128.0) * 0.90;
        kukaLWRVelLimit(3) = DEG2RAD(128.0) * 0.95;
        kukaLWRVelLimit(4) = DEG2RAD(204.0) * 0.95;
        kukaLWRVelLimit(5) = DEG2RAD(184.0) * 0.95;
        kukaLWRVelLimit(6) = DEG2RAD(184.0) * 0.95;
    }

    //prepare critical damping dynamics
    pArmCDDyn = new CDDynamics(
            nHandBaseId,            //DOF of arm
            UPDATE_INTERVAL,        //dt -- see Seungsu's catching package, target position was set in RobotUpdate function rather than CoreUpdate which might be more realtime intensive
            WN_JOINT_FILTER);
    pHandCDDyn = new CDDynamics(
            nJointNum - nHandBaseId,
            UPDATE_INTERVAL,
            WN_JOINT_FILTER);

    //init ros node
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "kukaAllegroSimControl", ros::init_options::NoSigintHandler);        //not to occupy Ctrl+C

    pRosNode = mLWRRobot->InitializeROS();
    //mSimWorldCmdPublisher = pRosNode->advertise< RobotGUI::SimWorldCmd >(SIM_WORLD_CMD, 5);
    mAllegroCmdSubscriber = pRosNode->subscribe(ALLEGRO_CMD_TOPIC, 5, &SimAllegroInterface::AllegroCmdSubCallback, this);

    mKukaCmdSubscriber = pRosNode->subscribe(KUKA_CMD_TOPIC, 5, &SimAllegroInterface::KukaCmdSubCallback, this);
    mObjPoseSubscriber = pRosNode->subscribe(OBJ_POSE_TOPIC, 5, &SimAllegroInterface::ObjPoseSubCallback, this);
    
    //load object
    int bSuccess = mObjData.LoadFromFile(OBJ_DATA_FILE);
    
    if(!bSuccess)
    {
        printf("Fail to load object data file...\n");
    }

    return STATUS_OK;
}
RobotInterface::Status SimAllegroInterface::RobotFree(){
    //release CDDyn
    if(pArmCDDyn != NULL)
    {
        delete pArmCDDyn;
        pArmCDDyn = NULL;
    }

    if(pHandCDDyn != NULL)
    {
        delete pHandCDDyn;
        pHandCDDyn = NULL;
    }

    //release ros node
    if(pRosNode != NULL)
    {
        pRosNode->shutdown();
        delete pRosNode;
        pRosNode = NULL;
    }

    nStateID = 1;
    bTopicReady = false;

    return STATUS_OK;
}
RobotInterface::Status SimAllegroInterface::RobotStart(){
    //initialize cddyn with current state
    mSensorsGroup.ReadSensors();

    //note that arm and hand are described in one structure file so the sensor actually returns full DOFs
    Vector initJoint = mSensorsGroup.GetJointAngles();

    pArmCDDyn->SetTarget(initJoint.GetSubVector(0, nHandBaseId));
    if(pHandCDDyn != NULL)
    {
        pHandCDDyn->SetTarget(initJoint.GetSubVector(nHandBaseId, nJointNum - nHandBaseId));
    }

    for(int ind = 0; ind < nHandBaseId; ++ind)
    {
        mArmJointTargetPosition[ind] = initJoint[ind];
    }
    for(int ind = 0; ind <  nJointNum - nHandBaseId; ++ind)
    {
        mHandJointTargetPosition[ind] = initJoint[nHandBaseId + ind];
    }

    mLastUpdateTime = ros::Time::now();

    return STATUS_OK;
}    
RobotInterface::Status SimAllegroInterface::RobotStop(){
    //note now the cddynamics state might not be synchronized, set back flag

    return STATUS_OK;
}
RobotInterface::Status SimAllegroInterface::RobotUpdate(){

    //record time
    ros::Time now = ros::Time::now();
    double dt = (now - mLastUpdateTime).toNSec() * 1e-9;

    mSensorsGroup.ReadSensors();
    Vector currentJointPos = mSensorsGroup.GetJointAngles();

    switch(nStateID)
    {
        case 0:
            //idle
            break;
        case 1:
            if(bStart)
            {
                if(ros::ok())
                {
                    Vector tarJoint = currentJointPos;

                    //pMutex->lock();
                    for(int ind = 0; ind < nHandBaseId; ++ind)
                    {
                        //tarJoint[ind] = curJoint[ind] + (mArmJointTargetPosition[ind] - curJoint[ind]) * GetClock().GetDt();
                        //restrict velocity, especially for arm...
                        switch(eMotionCmdType)
                        {
                            case MOTION_CMD_TYPE_VELOCITY:
                            {
                                double scalar = 20;
                                if(fabs(mArmJointTargetVel[ind]) > DEG2RAD(45.0))
                                {
                                    scalar = DEG2RAD(45.0) / fabs(mArmJointTargetVel[ind]);
                                }
                                tarJoint[ind] = currentJointPos[ind] + mArmJointTargetVel[ind] * dt * scalar;
                                break;
                            }
                            case MOTION_CMD_TYPE_POSITION:
                                tarJoint[ind] = mArmJointTargetPosition[ind];
                                break;
                            default:
                                tarJoint[ind] = currentJointPos[ind];
                        }
                    }
                    if(pArmCDDyn != NULL)
                    {
                    	//pArmCDDyn->SetWn(1.0 / dt);
                    	pArmCDDyn->SetWn(25.0);
                        pArmCDDyn->SetTarget(tarJoint.GetSubVector(0, nHandBaseId));
                    }

                    for(int ind = nHandBaseId; ind < nJointNum; ++ind)
                    {
                        switch(eMotionCmdType)
                        {
                            case MOTION_CMD_TYPE_VELOCITY:
                                tarJoint[ind] = currentJointPos[ind] + mHandJointTargetVel[ind] * dt;
                                break;
                            case MOTION_CMD_TYPE_POSITION:
                                tarJoint[ind] = mHandJointTargetPosition[ind - nHandBaseId];
                                break;
                            default:
                                tarJoint[ind] = currentJointPos[ind];
                        }
                    }
                    if(pHandCDDyn != NULL)
                    {
                        pHandCDDyn->SetTarget(tarJoint.GetSubVector(nHandBaseId, nJointNum - nHandBaseId));
                    }
                    //pMutex->unlock();
                    
                    //publish the message and write target position to joints...
                }
                else
                {
                    printf("ROS node is running unhappily...\n");
                    bStart = false;
                }
            }
            break;

        case 2:
            if(bStart)
            {
                //set target position and then leave it to CDS
                if(pArmCDDyn != NULL)
                {
                    pArmCDDyn->SetWn(WN_JOINT_PLANNER);
                    pArmCDDyn->SetTarget(mArmJointTargetPosition);
                }
                if(pHandCDDyn != NULL)
                {
                    pHandCDDyn->SetTarget(mHandJointTargetPosition);
                }
                nStateID = 0;
            }
            break;
            
    }
    mLastUpdateTime = now;

    if(bStart)
    {
        ros::spinOnce();
    }

    return STATUS_OK;
}
RobotInterface::Status SimAllegroInterface::RobotUpdateCore(){
   
    mSensorsGroup.ReadSensors();
    Vector currentJointPos = mSensorsGroup.GetJointAngles();
    //get filtered joint from CDS
    Vector tarJoint(nJointNum), tarArmJoint(nHandBaseId), tarHandJoint;
    //tarArmJoint.Resize(nHandBaseId);
    tarHandJoint.Resize(nJointNum - nHandBaseId);
 
    // check PArmCDDyn is initialized
    // if not
    // read joint real angle
    // set  pArmCDDyn->SetTarget() to ensure initial state of cddynamics is right
   
    if(!bArmCDDynReady)
    {
    	Vector curJnt(nHandBaseId);
        curJnt = currentJointPos.GetSubVector(0, nHandBaseId);
        pArmCDDyn->SetState(curJnt);
        bArmCDDynReady = true;
    }

    if(pArmCDDyn != NULL)
    {
        pArmCDDyn->Update();
        pArmCDDyn->GetState(tarArmJoint);
        tarJoint.SetSubVector(0, tarArmJoint);
    }

    if(!bHandCDDynReady)
    {
        Vector curJnt(16);
        curJnt = currentJointPos.GetSubVector(nHandBaseId, nJointNum - nHandBaseId);
        pHandCDDyn->SetState(curJnt);
        bHandCDDynReady = true;
    }
    if(pHandCDDyn != NULL)
    {
        //expand target joint vector...
        pHandCDDyn->Update();
        pHandCDDyn->GetState(tarHandJoint);
        //this module is for simulated robot, but for real robot, i think it should be okay to send command of more DOFs since LWRInterface will trunkthe length to LBR_MNJ: 7
        tarJoint.SetSubVector(nHandBaseId, tarHandJoint);
    }

	if(mLWRRobot->GetControlMode()!=Robot::CTRLMODE_POSITION)
		mLWRRobot->SetControlMode(Robot::CTRLMODE_POSITION);

    //tarJoint.Print();   
    mActuatorsGroup.SetJointAngles(tarJoint);
    mActuatorsGroup.WriteActuators();


    return STATUS_OK;
}
int SimAllegroInterface::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    if(cmd == "start")
    {
        if(pRosNode != NULL)
        {
            bStart = true;
            bTopicReady = true;
            nStateID = 1;
        }
    }
    else if(cmd == "stop")
    {
        bStart = false;
        nStateID = 0;
    }
    else if(cmd == "obj_frame")
    {
        //TODO: switch on/off displaying object frame
    }
    else if(cmd =="fk_index")
    {
        //get position of index finger tip
        int base_id = mLWRRobot->GetLinkIndex("TOOL");
        int index_tip_id = mLWRRobot->GetLinkIndex("link_3_0_tip");
        Vector3 origin(mLWRRobot->GetReferenceFrame(index_tip_id, base_id).GetOrigin());
        char msg[255];
        snprintf(msg, 255, "Position of index finger tip: %lf,  %lf, %lf", origin(0), origin(1), origin(2));
        GetConsole()->Print(msg);
    }
    else if(cmd =="fk_middle")
    {
        //get position of middle finger tip
        int base_id = mLWRRobot->GetLinkIndex("TOOL");
        int tip_id = mLWRRobot->GetLinkIndex("link_7_0_tip");
        Vector3 origin(mLWRRobot->GetReferenceFrame(tip_id, base_id).GetOrigin());
        char msg[255];
        snprintf(msg, 255, "Position of middle finger tip: %lf, %lf, %lf", origin(0), origin(1), origin(2));
        GetConsole()->Print(msg);
 
    }
    else if(cmd == "fk_thumb")
    {
        //get position of thumb finger tip
        int base_id = mLWRRobot->GetLinkIndex("TOOL");
        int tip_id = mLWRRobot->GetLinkIndex("link_15_0_tip");
        Vector3 origin(mLWRRobot->GetReferenceFrame(tip_id, base_id).GetOrigin());
        char msg[255];
        snprintf(msg, 255, "Position of index finger tip: %lf, %lf, %lf", origin(0), origin(1), origin(2));
        GetConsole()->Print(msg);
 
    }

    return 0;
}

void SimAllegroInterface::KukaCmdSubCallback(const sensor_msgs::JointState& msg)
{
    //try to write target position for joints of hand & arm
    int max_idx = msg.position.size() > nHandBaseId ? nHandBaseId : msg.position.size();
    
    for(int ind = 0; ind < max_idx; ++ind)
    {
        mArmJointTargetPosition[ind] = msg.position[ind];
    }
    if(msg.position.size() > nHandBaseId)
    {
        max_idx = msg.position.size() > nJointNum ? nJointNum : msg.position.size();
        for(int ind = nHandBaseId; ind < max_idx; ++ind)
        {
            mHandJointTargetPosition[ind - nHandBaseId] = msg.position[ind];
        }
    }

    //try to write target velocity for joints of hand & arm
    max_idx = msg.velocity.size() > nHandBaseId ? nHandBaseId : msg.velocity.size();
    //pMutex->lock();
    for(int ind = 0; ind < max_idx; ++ind)
    {
        mArmJointTargetVel[ind] = msg.velocity[ind];
    }
    if(msg.position.size() > nHandBaseId)
    {
        max_idx = msg.velocity.size() > nJointNum ? nJointNum : msg.velocity.size();
        for(int ind = nHandBaseId; ind < max_idx; ++ind)
        {
            mHandJointTargetVel[ind - nHandBaseId] = msg.velocity[ind];
        }
    }

    return;
}

void SimAllegroInterface::AllegroCmdSubCallback(const sensor_msgs::JointState& msg)
{
    unsigned int max_idx = msg.position.size() > mHandJointTargetPosition.Size() ? mHandJointTargetPosition.Size() : msg.position.size();
    for(unsigned int ind = 0; ind < max_idx; ++ind)
    {
        mHandJointTargetPosition[ind] = msg.position[ind];
    }
    //printf("get hand joint state.\n");
    return;
}

void SimAllegroInterface::ObjPoseSubCallback(const geometry_msgs::PoseStamped& msg)
{
    //draw a frame according to pose message in world frame
    //convert quaternion to matrix4
    Matrix4 ref;
    ref.Identity();
    
    Matrix3 rot;
    Vector quat(4);
    quat(0) = msg.pose.orientation.w;
    quat(1) = msg.pose.orientation.x;
    quat(2) = msg.pose.orientation.y;
    quat(3) = msg.pose.orientation.z;
    GetRotationMatrixFromQuaternion(quat, rot);
    
    Vector3 pos;
    pos(0) = msg.pose.position.x;
    pos(1) = msg.pose.position.y;
    pos(2) = msg.pose.position.z;

    ref.SetTranslation(pos);
    ref.SetOrientation(rot);

    //ref.Print();
    mObjPose.Set(ref);
    return;
}

void SimAllegroInterface::RobotDraw()
{
    GLTools::DrawRef(0.05, &mObjPose);
}

extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    SimAllegroInterface* create(){return new SimAllegroInterface();}
    void destroy(SimAllegroInterface* module){delete module;}
}

