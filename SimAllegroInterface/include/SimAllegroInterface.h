/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
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

/*===================================================================================
 * RobotToolKit simulator interface for allegro hand
 ===================================================================================*/ 

#ifndef SimAllegroInterface_H_
#define SimAllegroInterface_H_

#include "ros/ros.h"
#include "ros/service.h"
#include "ros/service_server.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/PoseStamped.h"
#include "RobotLib/RobotInterface.h"
#include "CDDynamics.h"
#include "LWRRobot.h"       //assume the base manipulator is kuka lwr

enum MotionCmdType
{
    MOTION_CMD_TYPE_POSITION,
    MOTION_CMD_TYPE_VELOCITY
};

class SimAllegroInterface : public RobotInterface
{
public:
            SimAllegroInterface();
    virtual ~SimAllegroInterface();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();
    virtual void                RobotDraw();            //override this function to draw the virtual frame

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);

protected:
    LWRRobot*                   mLWRRobot;
    MotionCmdType               eMotionCmdType;

    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;
    int                         nHandBaseId;
    int                         nJointNum;

    bool                        bStart;
    bool                        bTopicReady;
    int                         nStateID;

    Vector                      mHandJointTargetPosition;
    Vector                      mArmJointTargetPosition;
    Vector                      mHandJointTargetVel;
    Vector                      mArmJointTargetVel;

    ros::Time                   mLastUpdateTime;

    ros::NodeHandle*            pRosNode;
    ros::Publisher              mSimWorldCmdPublisher;
    ros::Subscriber             mAllegroCmdSubscriber;
    ros::Subscriber             mKukaCmdSubscriber;
    ros::Subscriber             mObjPoseSubscriber;

    virtual void                AllegroCmdSubCallback(const sensor_msgs::JointState& msg);
    virtual void                KukaCmdSubCallback(const sensor_msgs::JointState& msg);
    virtual void                ObjPoseSubCallback(const geometry_msgs::PoseStamped& msg);

    CDDynamics*                 pArmCDDyn;
    CDDynamics*                 pHandCDDyn;
    bool                        bArmCDDynReady;
    bool                        bHandCDDynReady;

    XmlTree                     mObjData;
    Matrix4                     mObjPose;
    bool                        bObjFrameOn;
};



#endif 
