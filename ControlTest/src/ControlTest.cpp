#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "StdTools/XmlTree.h"
#include "AllegroHandObjImpController.h"

/*=================================================
 *
 *  Test code
 *
 * ===============================================*/

#define ROBOT_CONFIG_FILE  "sim_kuka_allegro"
#define ALLEGRO_CMD_TOPIC     "/allegro/joint_cmd"
#define ALLEGRO_STATE_TOPIC   "/allegro/joint_state"
#define ALLEGRO_SIM_STATE   "/kuka_allegro/hand_joint_state"
#define OBJ_POSE_TOPIC      "/kuka_allegro/obj_pose"
#define ALLEGRO_DOF         16

using namespace std;

ros::NodeHandle*    nh;
ros::Subscriber     stateSub;
ros::Publisher      cmdPub;
ros::Publisher      statePub;
ros::Publisher      objPosePub;

sensor_msgs::JointState jointState;
sensor_msgs::JointState initGrasp;

void init_grasp()
{
    initGrasp.position.resize(ALLEGRO_DOF);
    //index
    initGrasp.position[0] = -0.0934;
    initGrasp.position[1] = 0.52913;
    initGrasp.position[2] = 0.78953;
    initGrasp.position[3] = 0.945820;
    //middle
    initGrasp.position[4] = -0.219948;
    initGrasp.position[5] = 0.5662826;
    initGrasp.position[6] = 0.766338;
    initGrasp.position[7] = 0.7694275;
    //thumb
    initGrasp.position[12] = -1.174159;
    initGrasp.position[13] = -0.479676;
    initGrasp.position[14] = 0.264248;
    initGrasp.position[15] = 0.680192;
    //tiny
    initGrasp.position[8] = 0.071099;
    initGrasp.position[9] = -0.00807;
    initGrasp.position[10] = 0.941599;
    initGrasp.position[11] = 0.791412;
}

void AllegroJointStateCallback(const sensor_msgs::JointState& msg)
{
    jointState = msg;
    return;
}

void ExtractFingerJoints(Vector& index, Vector& middle, Vector& thumb)
{
    //extract finger joints from join state variable
    index.Resize(4);    middle.Resize(4);   thumb.Resize(4);
    thumb(0) = jointState.position[12];  thumb(1) = jointState.position[13];  thumb(2) = jointState.position[14];  thumb(3) = jointState.position[15];
    index(0) = jointState.position[0];  index(1) = jointState.position[1];  index(2) = jointState.position[2];  index(3) = jointState.position[3];
    middle(0) = jointState.position[4]; middle(1) = jointState.position[5]; middle(2) = jointState.position[6]; middle(3) = jointState.position[7];
 
    return;
}

void SerializeFingerTorquesToMsg(const Vector& computed_trqs, sensor_msgs::JointState& msg)
{
    //fill joint state msg with specified torque command
    msg.effort.resize(ALLEGRO_DOF);
    
    //index
    
    msg.effort[0] = computed_trqs(0);   msg.effort[1] = computed_trqs(1);   msg.effort[2] = computed_trqs(2);   msg.effort[3] = computed_trqs(3);
    //middle
    msg.effort[4] = computed_trqs(4);   msg.effort[5] = computed_trqs(5);   msg.effort[6] = computed_trqs(6);   msg.effort[7] = computed_trqs(7);
    

    //tiny: all zero
    msg.effort[8] = 0.0;    msg.effort[9] = 0.0;    msg.effort[10] = 0.0;   msg.effort[11] = 0.0;
    //thumb: note rotation axis
    msg.effort[12] = computed_trqs(8);  msg.effort[13] = computed_trqs(9);  msg.effort[14] = computed_trqs(10); msg.effort[15] = computed_trqs(11);

    return;
}

int main(int argc, char** argv)
{
    init_grasp();

    string config_folder("./config/");
    string config_file(ROBOT_CONFIG_FILE);

    /*if there was specified config file, use this one...*/
    XmlTree args;
    XmlTree argStruct("args", "", 1, new XmlTree("config", "", "needArg=\"true\""));

    args.ParseArguments(argc, argv, &argStruct);

    if(args.Find("config") != NULL)
    {
        config_file = args.Find("config")->GetData();
    }

    string config_full_path = config_folder + config_file + ".xml";

    AllegroHandObjImpController controller(config_full_path.c_str());

    ros::init(argc, argv, "ControlTest", ros::init_options::NoSigintHandler);

    nh = new ros::NodeHandle();
    stateSub = nh->subscribe(ALLEGRO_STATE_TOPIC, 5, &AllegroJointStateCallback);
    cmdPub = nh->advertise< sensor_msgs::JointState >(ALLEGRO_CMD_TOPIC, 5);
    statePub = nh->advertise< sensor_msgs::JointState >(ALLEGRO_SIM_STATE, 5);
    objPosePub = nh->advertise< geometry_msgs::PoseStamped >(OBJ_POSE_TOPIC, 5);

    //test code here
    /*
    jointState.position.resize(16);

    for(int i = 0; i < jointState.position.size(); ++i)
    {
        jointState.position[i] = 0.0;
    }

    double thumb0[10] = { 0.4, 0.6, 0.8, 1.0, 1.0, 1.0, 1.0, 1.0, 0.6, 0.4 };
    double index1[10] = { 0.4, 0.6, 0.8, 1.0, 1.2, 1.0, 1.6, 1.0, 0.8, 0.4 };
    double middle1[10] = { 0.4, 0.6, 0.8, 1.0, 1.0, 1.0, 1.0, 1.0, 0.6, 0.4 };

    */
    //ObjState desiredObjState;
    /*
    for(int i = 0; i < 10; ++i)
    {
        getchar();
        //set each finger joint position
        Vector thumb_pos(4), index_pos(4), middle_pos(4);
        thumb_pos(0) = thumb0[i];
        thumb_pos(1) = thumb0[i];
        thumb_pos(2) = thumb0[i];
        index_pos(1) = index1[i];
        index_pos(2) = index1[i];
        middle_pos(1) = middle1[i];
        middle_pos(2) = middle1[i];
        controller.SetFingerChainState(0, index_pos);
        controller.SetFingerChainState(1, middle_pos);
        controller.SetFingerChainState(2, thumb_pos);
        jointState.position[1] = index1[i];
        jointState.position[2] = index1[i];
        jointState.position[5] = middle1[i];
        jointState.position[6] = middle1[i];
        jointState.position[12] = thumb0[i];
        jointState.position[13] = thumb0[i];
        jointState.position[14] = thumb0[i];
        //get current hand state and virtual obj frame
        statePub.publish(jointState);
        geometry_msgs::Pose poseMsg;
        ObjState objState;
        
        controller.GetObjCurrentState(objState);
        if(i == 0)
        {
            //use this as desired obj pose
            controller.SetDesiredObjState(objState);
        }

        ReferenceFrame baseFrame, objFrameWorld;
        controller.GetBaseRefFrame(baseFrame);
        objFrameWorld.Set(baseFrame.Mult(objState.mObjPose));
        
        Vector3 objPos(objFrameWorld.GetOrigin());
        Matrix3 objOri(objFrameWorld.GetOrient());
        //objPos.Print();
        //objOri.Print();
               
        Vector objOriQuat;
        objOri.GetQuaternionRepresentation(objOriQuat);
        poseMsg.position.x = objPos(0);
        poseMsg.position.y = objPos(1);
        poseMsg.position.z = objPos(2);

        poseMsg.orientation.w = objOriQuat(0);
        poseMsg.orientation.x = objOriQuat(1);
        poseMsg.orientation.y = objOriQuat(2);
        poseMsg.orientation.z = objOriQuat(3);

        objPosePub.publish(poseMsg);

        //calculate control signal
        Vector control;
//        printf("before update...\n");
        controller.Update();
//        printf("after update...\n");
        controller.GetControlOutput(control);
        printf("\nCalculated torque:\n");
        control.Print();
        printf("\n");

        ros::spinOnce();
    }*/
    bool bGraspDone = false;
    int nCycle = 0;
    ros::Rate r(0.3);       //0.3 Hz

#ifdef GMM_WORKSPACE_REACTION
    controller.InitGMMWorkspaceDesc(3, 22, "./SaveGMMFile/VirtualFrame_prio.txt", "./SaveGMMFile/VirtualFrame_mu.txt", "./SaveGMMFile/VirtualFrame_sigma.txt");
#endif

    while(ros::ok())
    {
        if(!bGraspDone)
        {
            printf("Press enter to grasp object...\n");
            getchar();
            sensor_msgs::JointState initCmd = initGrasp;
            cmdPub.publish(initCmd);
            bGraspDone = true;
            //wait a while
            printf("Grasping...\n");
            r.sleep();
            //get latest joint values 
            ros::spinOnce();
            printf("Press eneter to continue...\n");
            getchar();
            //use current virtual frame as initial desired obj pose
            Vector thumb_pos(4), index_pos(4), middle_pos(4);
            ExtractFingerJoints(index_pos, middle_pos, thumb_pos);
            controller.SetFingerChainState(0, index_pos);
            controller.SetFingerChainState(1, middle_pos);
            controller.SetFingerChainState(2, thumb_pos);
            ObjState objState;
    
            controller.GetObjCurrentState(objState);
            controller.SetDesiredObjState(objState);
            printf("Initial virtual frame:\n");
            objState.mObjPose.GetOrigin().Print();
            objState.mObjPose.GetOrient().Print();
            printf("############################\n");

            printf("Expected torques:\n");
            controller.Update();
            Vector computed_trqs;
            controller.GetControlOutput(computed_trqs);
            computed_trqs.Print();
        }

        //get current hand state and virtual obj frame
        geometry_msgs::PoseStamped poseMsg;
        ObjState objState;
        
        controller.GetObjCurrentState(objState);
        ReferenceFrame baseFrame, objFrameWorld;
        controller.GetBaseRefFrame(baseFrame);
        objFrameWorld.Set(baseFrame.Mult(objState.mObjPose));
        
        Vector3 objPos(objFrameWorld.GetOrigin());
        Matrix3 objOri(objFrameWorld.GetOrient());
        //objPos.Print();
        //objOri.Print();
               
        Vector objOriQuat;
        objOri.GetQuaternionRepresentation(objOriQuat);
        poseMsg.pose.position.x = objPos(0);
        poseMsg.pose.position.y = objPos(1);
        poseMsg.pose.position.z = objPos(2);

        poseMsg.pose.orientation.w = objOriQuat(0);
        poseMsg.pose.orientation.x = objOriQuat(1);
        poseMsg.pose.orientation.y = objOriQuat(2);
        poseMsg.pose.orientation.z = objOriQuat(3);
        
        //time stamp
        poseMsg.header.stamp = ros::Time::now();

        objPosePub.publish(poseMsg);

       
        //update model with sensory feedback
        Vector thumb_pos(4), index_pos(4), middle_pos(4);
        ExtractFingerJoints(index_pos, middle_pos, thumb_pos);
        controller.SetFingerChainState(0, index_pos);
        controller.SetFingerChainState(1, middle_pos);
        controller.SetFingerChainState(2, thumb_pos);
 
        controller.Update();
        Vector computed_trqs;
        controller.GetControlOutput(computed_trqs);
        //set zero torque command for kinethestic teaching
        computed_trqs.Zero();
        //computed_trqs.Print();
        sensor_msgs::JointState trqCmd;
        SerializeFingerTorquesToMsg(computed_trqs, trqCmd);

        cmdPub.publish(trqCmd);
/*
        if(nCycle > 25)
        {
            break;
        }
        ++nCycle;
        */
        ros::spinOnce();
        //getchar();
    }

    nh->shutdown();
    delete nh;
    nh = NULL;

    return 0;
}
