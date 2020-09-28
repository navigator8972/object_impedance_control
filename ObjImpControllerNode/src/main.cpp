/*node to provide interface for object impedance controller*/
/*============================================================================
 * 
 *  Provide service for switching control mode and home operation
 *  Subscribe/Publish message for specifying desired control target
 *
 *============================================================================ */
#include "ros/ros.h"
#include <queue>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
#include "ObjImpManipulation/MultiFingersObjImpCtrlCmd.h"
#include "ObjImpManipulation/ObjImpManipulationCmd.h"
#include "StdTools/XmlTree.h"
#include "AllegroHandObjImpController.h"
#include "MotionGenerators/CDDynamics.h"
#include "SynTouchContact/SynTac.h"

#define QUATERNION_SLERP

#define HAND_JOINT_STATE_TOPIC              "/allegro/joint_state"
#define HAND_JOINT_CMD_TOPIC                "/allegro/joint_cmd"
#define OBJ_IMPCTRL_CMD_TOPIC               "/objimpctrl/obj_pose_cmd"
#define OBJ_IMPCTRL_OBJ_POSE_STATE_TOPIC    "/objimpctrl/obj_pose_state"
#define OBJ_IMPCTRL_OBJ_POSE_STATE_TOPIC_NoTac    "/objimpctrl/obj_pose_state_NoTac"

#define OBJ_IMPCTRL_CMD_SRV                 "objimpctrl/node_cmd"
#define HAND_DOF_NUM                        16

#define CONTACT1_TOPIC                 "/contactF1"
#define CONTACT2_TOPIC                 "/contactF2"
#define CONTACT3_TOPIC                 "/contactF3"

using namespace MathLib;

CDDynamics*         pHandCDDyn;
bool                bCDDynInit = false;

ros::NodeHandle*    nh = NULL;
ros::Subscriber     handStateSub;
ros::Subscriber     objCtrlCmdSub;

ros::Publisher      objPosePub;
ros::Publisher      objPosePubNoTac;   //Miao

ros::Publisher      handCmdPub;

ros::Publisher      ContactF1;    //miao
ros::Publisher      ContactF2;   ///miao
ros::Publisher      ContactF3;  ///miao

ros::Subscriber     contactPntF1Sub;
ros::Subscriber     contactPntF2Sub;
ros::Subscriber     contactPntF3Sub;

ros::ServiceServer  nodeCmdSrv;

/*specifying the meaning of msg, incremental or absolute*/
bool bGraspImpIncrmnt = false;
bool bGraspRestLenIncrmnt = false;
bool bObjImpIncrmnt = false;
bool bObjPoseIncrmnt = false; // default no increamental

bool bJntInitialized = false;

enum CTRL_MODE
{
	CTRL_MODE_JOINT_POSITION,
	CTRL_MODE_OBJECT_IMPEDANCE,
	CTRL_MODE_KINESTHETIC_TEACHING
};
CTRL_MODE eCtrlMode = CTRL_MODE_JOINT_POSITION;

const char*                     sHandConfigFile = "sim_kuka_allegro";
sensor_msgs::JointState         homeJointState;
sensor_msgs::JointState         currJointState;
sensor_msgs::JointState         graspJointState;
sensor_msgs::JointState         tarJointState;
AllegroHandObjImpController*    pImpCtrl = NULL;

//a small stiffness controller for pinky, we need to keep its position when object impedance controller is active
double dPinkyGain = 1.0;
sensor_msgs::JointState		restJointState;

queue< Vector3 > contactPosQue[3];
queue< Vector3 > contactForceQue[3];
const int nQueFilterLen = 10;
Vector3 avgCntctPnt[3];
Vector3 avgCntctForce[3];

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
#ifdef QUATERNION_SLERP
bool bQuatSlerpActive = false;
double lambda = 0.0;
Vector quat1(4);
Vector quat2(4);
//slerp quaternion interpolation
void QuaternionSlerp(const Vector& q1, const Vector& q2, double lambda, Vector& qr)
{
	/*
    if(lambda > 1 || lambda < 0)
    {
        //invalid lambda
        return;
    }
	 */
	//algorithm from www.sonycsl.co.jp/person/nielsen/visualcomputing/programs/slerp.cpp
	double dotproduct = q1(1) * q2(1) + q1(2) * q2(2) + q1(3) * q2(3) + q1(0) * q2(0);
	//lambda = lambda / 2.0;

	double theta = acos(dotproduct);
	if(theta < 0.0)
	{
		theta = -theta;
	}
	double st = sin(theta);
	double sut = sin(lambda * theta);
	double sout = sin((1 - lambda) * theta);
	double coeff1 = sout / st;
	double coeff2 = sut / st;

	qr.Resize(4);
	for(int i = 0; i < 4; ++i)
	{
		qr(i) = coeff1 * q1(i) + coeff2 * q2(i);
	}
	//normalization
	qr = qr / qr.Norm();
	return;
}
#endif

void UpdateCDDynamicsPlanner(sensor_msgs::JointState target)
{
	if(pHandCDDyn != NULL)
	{
		//update CDDynamics current state with current joint state
		Vector tmpCurrJointState(HAND_DOF_NUM);
		tmpCurrJointState.Set(&(currJointState.position[0]), HAND_DOF_NUM);
		pHandCDDyn->SetState(tmpCurrJointState);

		//update CDDynamics target state with specified joint state
		Vector tmpTarJointState(HAND_DOF_NUM);
		tmpTarJointState.Set(&(target.position[0]), HAND_DOF_NUM);
		pHandCDDyn->SetTarget(tmpTarJointState);
	}
	return;
}

#define DEG2RAD(x)      ( x * 3.141592658 / 180.0 )

void InitializeImpController(int argc, char** argv)
{
	//home position for hand for grasp correction test 02.08.2014
	homeJointState.position.resize(HAND_DOF_NUM);

	homeJointState.position[0] = 0.003103557499952344-0.2;
	homeJointState.position[1] = 0.9602463925377912;
	homeJointState.position[2] = 0.2329695760077956;
	homeJointState.position[3] = 0.06430089749072948;

	homeJointState.position[4] = -0.1094039293915102-0.2;
	homeJointState.position[5] = 0.8372023964890149;
	homeJointState.position[6] = 0.47803902997068883;
	homeJointState.position[7] = 0.2568160710372291;

	homeJointState.position[8] = -0.18515994995006335;
	homeJointState.position[9] = 0.10953373836930881;
	homeJointState.position[10] = -0.25013458243493697;
	homeJointState.position[11] = -0.0913372907471789;

	homeJointState.position[12] = -1.3753299335570754;
	homeJointState.position[13] = -0.2110010457648356+0.2;
	homeJointState.position[14] = 0.2381257010122454;
	homeJointState.position[15] = 0.43691985354002044;

	/*
    homeJointState.position[0] = 1.5 * 0.0934;
    homeJointState.position[1] = 0.52913;
    homeJointState.position[2] = 0.78953;
    homeJointState.position[3] = 0.945820;

    homeJointState.position[4] = 2.5 * 0.119948;
    homeJointState.position[5] = 0.5662826;
    homeJointState.position[6] = 0.766338;
    homeJointState.position[7] = 0.7694275;

    homeJointState.position[12] = -1.174159;
    homeJointState.position[13] = -0.479676;
    homeJointState.position[14] = 0.064248;
    homeJointState.position[15] = 0.680192;
	 */

	/*
	//this is home position for bulb inserting
	homeJointState.position[0] = -0.017;//DEG2RAD(4.0);
	homeJointState.position[1] = 1.257;//DEG2RAD(27.5);
	homeJointState.position[2] = 0.2844;//DEG2RAD(71.4);
	homeJointState.position[3] = 0.27166;//DEG2RAD(16.9);

	homeJointState.position[4] = -0.1;//DEG2RAD(4.0);
	homeJointState.position[5] = 1.3183;//DEG2RAD(43.0);
	homeJointState.position[6] = 0.3116;//DEG2RAD(60.0);
	homeJointState.position[7] = 0.0726;//DEG2RAD(6.0);

	homeJointState.position[12] = -0.824 - 0.17453;//DEG2RAD(-27.0);
	homeJointState.position[13] = -0.315;//DEG2RAD(-62.0);  bulb -0.715
	homeJointState.position[14] = 0.0356;//DEG2RAD(26.4);
	homeJointState.position[15] = 0.5047;//DEG2RAD(49.3);

	homeJointState.position[8] = 0.071099;
	homeJointState.position[9] = -0.00807;
	homeJointState.position[10] = 0.941599;
	homeJointState.position[11] = 0.791412;
	 */

	//this is home position for handwriting
	/*
    homeJointState.position[0] = 0.59597;//DEG2RAD(4.0);
    homeJointState.position[1] = 1.0927;//DEG2RAD(27.5);
    homeJointState.position[2] = 0.60517;//DEG2RAD(71.4);
    homeJointState.position[3] = 0.00697;//DEG2RAD(16.9);

    homeJointState.position[4] = 0.500808;//DEG2RAD(4.0);
    homeJointState.position[5] = 0.99634;//DEG2RAD(43.0);
    homeJointState.position[6] = 0.286484;//DEG2RAD(60.0);
    homeJointState.position[7] = 0.3981312;//DEG2RAD(6.0);

    homeJointState.position[12] = -1.3665088;//DEG2RAD(-27.0);
    homeJointState.position[13] = -1.271265;//DEG2RAD(-62.0);
    homeJointState.position[14] = 0.52175;//DEG2RAD(26.4);
    homeJointState.position[15] = 0.52077;//DEG2RAD(49.3);

    homeJointState.position[8] = 0.423756;
    homeJointState.position[9] = 0.20291258;
    homeJointState.position[10] = 0.8735182;
    homeJointState.position[11] = -0.207350;
	 */

	// home position for different grasps:
	// grasp 1
	/*
	    homeJointState.position[0] = 0.00523-0.425;
	    homeJointState.position[1] = 0.7383;
	    homeJointState.position[2] = 0.840+0.1;
	    homeJointState.position[3] = 0.0138;

	    homeJointState.position[4] = 0.0265-0.375;
	    homeJointState.position[5] = 0.6215;
	    homeJointState.position[6] = 0.8121+0.1;
	    homeJointState.position[7] = 0.3809;

	    homeJointState.position[8] = 0.0750-0.395;
	    homeJointState.position[9] = 0.7071;
	    homeJointState.position[10] = 0.8706+0.1;
	    homeJointState.position[11] = 0.0629;

	    homeJointState.position[12] = -1.2788+0.4;
	    homeJointState.position[13] =  0.0895;
	    homeJointState.position[14] = -0.150+0.2;
	    homeJointState.position[15] = 0.7753;
	 */
	//grasp 2
	/*
	    homeJointState.position[0] = 0.0219-0.425;
	    homeJointState.position[1] = 0.6657+0.2;
	    homeJointState.position[2] = 0.7105;
	    homeJointState.position[3] = -0.02477;

	    homeJointState.position[4] = -0.1447-0.375;
	    homeJointState.position[5] = 0.6436+0.1;
	    homeJointState.position[6] = 0.7704;
	    homeJointState.position[7] = 0.016;

	    homeJointState.position[8] = 0.154-0.395;
	    homeJointState.position[9] = 0.6833;
	    homeJointState.position[10] = 0.7421;
	    homeJointState.position[11] = 0.0543;

	    homeJointState.position[12] = -1.31+0.4;
	    homeJointState.position[13] = -0.352;
	    homeJointState.position[14] = 0.2981;
	    homeJointState.position[15] = 0.6815;

	 */

	//the initial grasp joint state is same as home joint state
	graspJointState = homeJointState;


	//use specified config file to initialize object impedance controller
	string config_folder("./config/");
	string config_file(sHandConfigFile);

	/*if there was specified config file, use this one...*/
	XmlTree args;
	XmlTree argStruct("args", "", 1, new XmlTree("config", "", "needArg=\"true\""));

	args.ParseArguments(argc, argv, &argStruct);

	if(args.Find("config") != NULL)
	{
		config_file = args.Find("config")->GetData();
	}

	string config_full_path = config_folder + config_file + ".xml";
	pImpCtrl = new AllegroHandObjImpController(config_full_path.c_str());

	if(pImpCtrl == NULL)
	{
		ROS_INFO("Fail to initialize impedance controller from %s.\n", config_file.c_str());
	}
	else
	{
		ROS_INFO("Successfully initialize impedance controller.\n");
	}

	return;
}

//extract finger joints from hand joint state, can have different implementation for different hands
void ExtractFingerJoints(Vector& index, Vector& middle, Vector& thumb)
{
	//extract finger joints from join state variable, for allegro
	index.Resize(4);    middle.Resize(4);   thumb.Resize(4);
	thumb(0) = currJointState.position[12];  thumb(1) = currJointState.position[13];  thumb(2) = currJointState.position[14];  thumb(3) = currJointState.position[15];
	index(0) = currJointState.position[0];  index(1) = currJointState.position[1];  index(2) = currJointState.position[2];  index(3) = currJointState.position[3];
	middle(0) = currJointState.position[4]; middle(1) = currJointState.position[5]; middle(2) = currJointState.position[6]; middle(3) = currJointState.position[7];

	return;
}


void SerializeFingerTorquesToMsg(const Vector& computed_trqs, sensor_msgs::JointState& msg)
{
	//fill joint state msg with specified torque command, for allegro
	msg.effort.resize(HAND_DOF_NUM);

	//index
	msg.effort[0] = computed_trqs(0);   msg.effort[1] = computed_trqs(1);   msg.effort[2] = computed_trqs(2);   msg.effort[3] = computed_trqs(3);
	//middle
	msg.effort[4] = computed_trqs(4);   msg.effort[5] = computed_trqs(5);   msg.effort[6] = computed_trqs(6);   msg.effort[7] = computed_trqs(7);

	//tiny: all zero
	//msg.effort[8] = 0.0;    msg.effort[9] = 0.0;    msg.effort[10] = 0.0;   msg.effort[11] = 0.0;
	//<hyin/Aug-06-2014> keep current torque for pinky...
	//use a small stiffness controller for controlling pinky
	for(int i = 8; i < 12; ++i)
	{
		//msg.effort[8] = restJointState.effort[8] + restJointState.position[8] - currJointState.position[8];	msg.effort[9] = currJointState.effort[9];	msg.effort[10] = currJointState.effort[10];	msg.effort[11] = currJointState.effort[11];
		msg.effort[i] = restJointState.effort[i] + (restJointState.position[i] - currJointState.position[i]) * dPinkyGain;
	}
	//thumb: note rotation axis
	msg.effort[12] = computed_trqs(8);  msg.effort[13] = computed_trqs(9);  msg.effort[14] = computed_trqs(10); msg.effort[15] = computed_trqs(11);

	return;
}


void HandJointStateCallback(const sensor_msgs::JointState& msg)
{
	currJointState = msg;
	bJntInitialized = true;
	if(!bCDDynInit)
	{
		//initialize CDDynamics with current joint state
		UpdateCDDynamicsPlanner(currJointState);
		bCDDynInit = true;
	}
	return;
}

void ObjImpCtrlCmdHandler(const ObjImpManipulation::MultiFingersObjImpCtrlCmd& msg)
{
	if(pImpCtrl == NULL)
	{
		return;
	}
	//for obj pose
	if(!msg.obj_pose.empty())
	{
		ObjState objState;
		pImpCtrl->GetDesiredObjState(objState);
		Vector3 pos(objState.mObjPose.GetOrigin());
		if(bObjPoseIncrmnt)
		{
			pos(0) = pos(0) + msg.obj_pose[0].position.x;
			pos(1) = pos(1) + msg.obj_pose[0].position.y;
			pos(2) = pos(2) + msg.obj_pose[0].position.z;
		}
		else
		{
			pos(0) = msg.obj_pose[0].position.x;
			pos(1) = msg.obj_pose[0].position.y;
			pos(2) = msg.obj_pose[0].position.z;
		}
		objState.mObjPose.SetOrigin(pos);

		//check if rotation quaternion is valid
		Vector quat(4);
		quat(0) = msg.obj_pose[0].orientation.w;
		quat(1) = msg.obj_pose[0].orientation.x;
		quat(2) = msg.obj_pose[0].orientation.y;
		quat(3) = msg.obj_pose[0].orientation.z;

		//printf("Specified quaternion: %lf, %lf, %lf, %lf\n", quat(0), quat(1), quat(2), quat(3));
		//printf("Norm of quaternion: %lf\n", quat.Norm());
		if(quat.Norm() > 0.99 && quat.Norm() < 1.01)
		{
			//convert this to a rotation matrix
			Matrix3 desiredRot;
			GetRotationMatrixFromQuaternion(quat, desiredRot);
			if(bObjPoseIncrmnt)
			{
				Matrix3 rot;
				desiredRot.Mult(objState.mObjPose.GetOrient(), rot);
				objState.mObjPose.SetOrient(rot);
			}
			else
			{
				objState.mObjPose.SetOrient(desiredRot);
			}
#ifdef QUATERNION_SLERP
			//update quat1 & quat2
			ObjState currObjState;
			if(pImpCtrl != NULL)
			{
				//update model with sensory feedback
				Vector thumb_pos(4), index_pos(4), middle_pos(4);
				ExtractFingerJoints(index_pos, middle_pos, thumb_pos);
				pImpCtrl->SetFingerChainState(0, index_pos);
				pImpCtrl->SetFingerChainState(1, middle_pos);
				pImpCtrl->SetFingerChainState(2, thumb_pos);
				pImpCtrl->Update();

				pImpCtrl->GetObjCurrentState(currObjState);
				Matrix3 currRot(currObjState.mObjPose.GetOrient());
				currRot.GetQuaternionRepresentation(quat1);
				desiredRot.GetQuaternionRepresentation(quat2);
				lambda = 0.0;
				bQuatSlerpActive = true;
			}
#endif
		}
		else
		{
			//invalid quaternion specified, ignore it
			printf("Specified quaternion is not normalized.\n");
		}
		ROS_INFO("Update desired object pose...\n");
		objState.mObjPose.GetOrigin().Print();
		pImpCtrl->SetDesiredObjState(objState);
	}

	//for obj stiffness
	if(!msg.obj_trans_stiff.empty())
	{
		ImpedanceParms tmpObjImpParm;
		pImpCtrl->GetDesiredObjImp(tmpObjImpParm);
		//only care about diagonal entries
		if(bObjImpIncrmnt)
		{
			tmpObjImpParm.mStiffMatrixTrans(0, 0) = tmpObjImpParm.mStiffMatrixTrans(0, 0) + msg.obj_trans_stiff[0].x;
			tmpObjImpParm.mStiffMatrixTrans(1, 1) = tmpObjImpParm.mStiffMatrixTrans(1, 1) + msg.obj_trans_stiff[0].y;
			tmpObjImpParm.mStiffMatrixTrans(2, 2) = tmpObjImpParm.mStiffMatrixTrans(2, 2) + msg.obj_trans_stiff[0].z;
		}
		else
		{
			tmpObjImpParm.mStiffMatrixTrans(0, 0) = msg.obj_trans_stiff[0].x;
			tmpObjImpParm.mStiffMatrixTrans(1, 1) = msg.obj_trans_stiff[0].y;
			tmpObjImpParm.mStiffMatrixTrans(2, 2) = msg.obj_trans_stiff[0].z;

			ROS_INFO("Update translational stiffness with %lf, %lf, %lf\n", msg.obj_trans_stiff[0].x, msg.obj_trans_stiff[0].y, msg.obj_trans_stiff[0].z);
		}

		pImpCtrl->SetDesiredObjImp(tmpObjImpParm);
		/*
       pImpCtrl->GetDesiredObjImp(tmpObjImpParm);
       tmpObjImpParm.mStiffMatrixTrans.Print();
		 */

	}
	if(!msg.obj_rot_stiff.empty())
	{
		ImpedanceParms tmpObjImpParm;
		pImpCtrl->GetDesiredObjImp(tmpObjImpParm);
		//only care about diagonal entries
		if(bObjImpIncrmnt)
		{
			tmpObjImpParm.mStiffMatrixRot(0, 0) = tmpObjImpParm.mStiffMatrixRot(0, 0) + msg.obj_rot_stiff[0].x;
			tmpObjImpParm.mStiffMatrixRot(1, 1) = tmpObjImpParm.mStiffMatrixRot(1, 1) + msg.obj_rot_stiff[0].y;
			tmpObjImpParm.mStiffMatrixRot(2, 2) = tmpObjImpParm.mStiffMatrixRot(2, 2) + msg.obj_rot_stiff[0].z;
		}
		else
		{
			tmpObjImpParm.mStiffMatrixRot(0, 0) = msg.obj_rot_stiff[0].x;
			tmpObjImpParm.mStiffMatrixRot(1, 1) = msg.obj_rot_stiff[0].y;
			tmpObjImpParm.mStiffMatrixRot(2, 2) = msg.obj_rot_stiff[0].z;

			ROS_INFO("Update rotational stiffness with %lf, %lf, %lf\n", msg.obj_rot_stiff[0].x, msg.obj_rot_stiff[0].y, msg.obj_rot_stiff[0].z);
		}

		pImpCtrl->SetDesiredObjImp(tmpObjImpParm);
		/*
        pImpCtrl->GetDesiredObjImp(tmpObjImpParm);
        tmpObjImpParm.mStiffMatrixRot.Print();
		 */
	}

	//for grasp stiff
	if(!msg.grasp_stiff.empty())
	{
		for(int i = 0; i < msg.grasp_stiff.size(); ++i)
		{
			ImpedanceParms tmpFingerImpParm;
			pImpCtrl->GetDesiredFingerImpedance(i, tmpFingerImpParm);

			if(bGraspImpIncrmnt)
			{
				tmpFingerImpParm.mStiffMatrixTrans(0, 0) = tmpFingerImpParm.mStiffMatrixTrans(0, 0) + msg.grasp_stiff[i].x;
				tmpFingerImpParm.mStiffMatrixTrans(1, 1) = tmpFingerImpParm.mStiffMatrixTrans(1, 1) + msg.grasp_stiff[i].y;
				tmpFingerImpParm.mStiffMatrixTrans(2, 2) = tmpFingerImpParm.mStiffMatrixTrans(2, 2) + msg.grasp_stiff[i].z;
			}
			else
			{
				tmpFingerImpParm.mStiffMatrixTrans(0, 0) = msg.grasp_stiff[i].x;
				tmpFingerImpParm.mStiffMatrixTrans(1, 1) = msg.grasp_stiff[i].y;
				tmpFingerImpParm.mStiffMatrixTrans(2, 2) = msg.grasp_stiff[i].z;

				ROS_INFO("Update grasp stiffness with %lf, %lf, %lf\n", msg.grasp_stiff[i].x, msg.grasp_stiff[i].y, msg.grasp_stiff[i].z);
			}

			pImpCtrl->SetDesiredFingerImpedance(i, tmpFingerImpParm);
			/*
            pImpCtrl->GetDesiredFingerImpedance(i, tmpFingerImpParm);
            tmpFingerImpParm.mStiffMatrixTrans.Print();
			 */
		}
	}
	//for grasp rest length ratio
	if(!msg.grasp_rest_len_ratio.empty())
	{
		for(int i = 0; i < msg.grasp_rest_len_ratio.size(); ++i)
		{
			pImpCtrl->SetContactStiffRestRatio(i, msg.grasp_rest_len_ratio[i]);
		}
	}

	//Miao Grasp correction

	if(!msg.FinCorrectionStiffness.empty()){


		for(int i=0; i<msg.FinCorrectionStiffness.size();i++){

			if(msg.FinCorrectionStiffness[i]<-0.5 && msg.FinCorrectionStiffness[i]>-1.5){      //ignore all

				continue;

			}else if(msg.FinCorrectionStiffness[i]<-1.5&&msg.FinCorrectionStiffness[i]>-2.5){  // only change position

				if(msg.FinDesiredCorrection.size()>i){
					Vector3 tmp;
					tmp[0]=msg.FinDesiredCorrection[i].x;
					tmp[1]=msg.FinDesiredCorrection[i].y;
					tmp[2]=msg.FinDesiredCorrection[i].z;

					pImpCtrl->SetCorrectionPosition(i,tmp);

				}
			}else{

				pImpCtrl->SetCorrectionStiffness(i,msg.FinCorrectionStiffness[i]);

				if(msg.FinDesiredCorrection.size()>i){
					Vector3 tmp;
					tmp[0]=msg.FinDesiredCorrection[i].x;
					tmp[1]=msg.FinDesiredCorrection[i].y;
					tmp[2]=msg.FinDesiredCorrection[i].z;
					pImpCtrl->SetCorrectionPosition(i,tmp);
				}

			}



		}

	}
	//Miao

	//check current and desired virtual frame
	/*
    ObjState currState, desiredState;
    pImpCtrl->GetObjCurrentState(currState);
    pImpCtrl->GetDesiredObjState(desiredState);
    printf("Current virtual frame:\n");
    currState.mObjPose.GetOrigin().Print();
    currState.mObjPose.GetOrient().Print();
    printf("Desired virtual frame:\n");
    desiredState.mObjPose.GetOrigin().Print();
    desiredState.mObjPose.GetOrient().Print();
	 */

	return;
}

void ContactCallbackHelper(int i, const SynTouchContact::SynTac& msg)
{
	//update force queue as well as averaged value
	Vector3 newForceData;
	newForceData(0) = msg.fx;
	newForceData(1) = msg.fy;
	newForceData(2) = msg.fz;
	Vector3 oldForceData(contactForceQue[i].front());
	avgCntctForce[i] *= nQueFilterLen;
	avgCntctForce[i] -= oldForceData;
	avgCntctForce[i] += newForceData;
	avgCntctForce[i] /= nQueFilterLen;
	contactForceQue[i].pop();
	contactForceQue[i].push(newForceData);
	//update corresponding queue as well as averaged value
	Vector3 newData;
	if(newForceData.Norm() > 0.1)
	{
		//warning: the message use unit of mm
		newData(0) = msg.x / 1000.0;
		newData(1) = msg.y / 1000.0;
		newData(2) = msg.z / 1000.0;
	}
	else
	{
		//it seems the default position for syntouch contact message is zero. this is just a double guarantee...
		newData.Zero();
	}
	Vector3 oldData(contactPosQue[i].front());
	avgCntctPnt[i] *= nQueFilterLen;
	avgCntctPnt[i] -= oldData;
	avgCntctPnt[i] += newData;
	avgCntctPnt[i] /= nQueFilterLen;
	contactPosQue[i].pop();
	contactPosQue[i].push(newData);
	return;
}

void ContactF1Callback(const SynTouchContact::SynTac& msg)
{
	ContactCallbackHelper(0, msg);
	return;
}

void ContactF2Callback(const SynTouchContact::SynTac& msg)
{
	ContactCallbackHelper(1, msg);
	return;
}

void ContactF3Callback(const SynTouchContact::SynTac& msg)
{
	ContactCallbackHelper(2, msg);
	return;
}

void ObjImpCtrlCmdCallback(const ObjImpManipulation::MultiFingersObjImpCtrlCmd& msg)
{
	ObjImpCtrlCmdHandler(msg);
	return;
}


bool ObjImpCtrlRequestHandler(ObjImpManipulation::ObjImpManipulationCmd::Request& req, ObjImpManipulation::ObjImpManipulationCmd::Response& res)
{
	if(req.cmd.compare("home") == 0)
	{
		//home
		ROS_INFO("Received request of home...\n");
		//ignore pinky finger
		tarJointState = homeJointState;
		//**** pinky finger
		//tarJointState.position[8] = currJointState.position[8];
		//tarJointState.position[9] = currJointState.position[9];
		//tarJointState.position[10] = currJointState.position[10];
		//tarJointState.position[11] = currJointState.position[11];

		UpdateCDDynamicsPlanner(tarJointState);
		//handCmdPub.publish(homeJointCmd);
		eCtrlMode = CTRL_MODE_JOINT_POSITION;
		res.response = "OK";
	}
	else if(req.cmd.compare("obj_pose_inc") == 0)
	{
		if(bObjPoseIncrmnt)
		{
			ROS_INFO("Received request of switching off specifying object pose incrementally...\n");
			bObjPoseIncrmnt = false;
		}
		else
		{
			ROS_INFO("Received request of switching on specifying object pose incrementally...\n");
			bObjPoseIncrmnt = true;
		}
		res.response = "OK";
	}
	else if(req.cmd.compare("obj_imp_inc") == 0)
	{
		if(bObjImpIncrmnt)
		{
			ROS_INFO("Received request of switching off specifying object impedance incrementally...\n");
			bObjImpIncrmnt = false;
		}
		else
		{
			ROS_INFO("Received request of switching on specifying object impedance incrementally...\n");
			bObjImpIncrmnt = true;
		}
		res.response = "OK";
	}
	else if(req.cmd.compare("grasp_imp_inc") == 0)
	{
		if(bGraspImpIncrmnt)
		{
			ROS_INFO("Received request of switching off specifying grasp impedance incrementally...\n");
			bGraspImpIncrmnt = false;
		}
		else
		{
			ROS_INFO("Received request of switching on specifying grasp impedance incrementally...\n");
			bGraspImpIncrmnt = true;
		}
		res.response = "OK";
	}
	else if(req.cmd.compare("pos_mode") == 0)
	{
		ROS_INFO("Received request of setting control mode to joint position...\n");
		if(eCtrlMode != CTRL_MODE_JOINT_POSITION)
		{
			eCtrlMode = CTRL_MODE_JOINT_POSITION;
			//set current joint position
			handCmdPub.publish(currJointState);
		}
		res.response = "OK";
	}
	else if(req.cmd.compare("imp_mode") == 0)
	{
		ROS_INFO("Received request of setting control mode to object impedance...\n");
		if(eCtrlMode != CTRL_MODE_OBJECT_IMPEDANCE)
		{
			eCtrlMode = CTRL_MODE_OBJECT_IMPEDANCE;
			//set current object pose as desired one
			ObjState currObjState;
			if(pImpCtrl != NULL)
			{
				//update model with sensory feedback
				Vector thumb_pos(4), index_pos(4), middle_pos(4);
				ExtractFingerJoints(index_pos, middle_pos, thumb_pos);
				pImpCtrl->SetFingerChainState(0, index_pos);
				pImpCtrl->SetFingerChainState(1, middle_pos);
				pImpCtrl->SetFingerChainState(2, thumb_pos);
				pImpCtrl->Update();

				pImpCtrl->GetObjCurrentState(currObjState);
				/*
                currObjState.mObjPose.GetOrigin().Print();
                currObjState.mObjPose.GetOrient().Print();
				 */
				pImpCtrl->SetDesiredObjState(currObjState);

				//update joint state at this time instant for controlling pinky finger
				restJointState = currJointState;
				//compensate a bit to ensure the contact of pinky
				// miao
				//restJointState.position[9] += 0.3;
			}
		}
		res.response = "OK";
	}
	else if(req.cmd.compare("kine_mode") == 0)
	{
		//kinesthetic teaching mode
		ROS_INFO("Received request of setting control mode to kinesthetic teaching...\n");
		if(eCtrlMode != CTRL_MODE_KINESTHETIC_TEACHING)
		{
			eCtrlMode = CTRL_MODE_KINESTHETIC_TEACHING;
			sensor_msgs::JointState zeroTrq;
			zeroTrq.effort.resize(HAND_DOF_NUM);
			for(int i = 0; i < zeroTrq.effort.size(); ++i)
			{
				zeroTrq.effort[i] = 0;
			}
			handCmdPub.publish(zeroTrq);
		}
		res.response = "OK";
	}
	else if(req.cmd.compare("rec_grasp") == 0)
	{
		//record joint configuration for grasping
		ROS_INFO("Received request of updating record of grasp...\n");
		graspJointState = currJointState;
		res.response = "OK";
	}
	else if(req.cmd.compare("grasp") == 0)
	{
		//drive to grasp joint state
		ROS_INFO("Received request of moving to grasping posture...\n");
		//handCmdPub.publish(graspJointState);
		UpdateCDDynamicsPlanner(graspJointState);
		eCtrlMode = CTRL_MODE_JOINT_POSITION;
		res.response = "OK";
	}
	else if(req.cmd.compare("release") == 0)
	{
		//release grasp, substract a small value to finger proximal joints
		ROS_INFO("Received request of releasing grasp...\n");
		tarJointState = currJointState;
		tarJointState.position[1] = tarJointState.position[1] - 0.2;
		tarJointState.position[2] = tarJointState.position[2] - 0.2;
		tarJointState.position[3] = tarJointState.position[3] - 0.2;
		tarJointState.position[5] = tarJointState.position[5] - 0.2;
		tarJointState.position[6] = tarJointState.position[6] - 0.2;
		tarJointState.position[7] = tarJointState.position[7] - 0.2;
		tarJointState.position[14] = tarJointState.position[14] - 0.2;
		//handCmdPub.publish(relJointState);
		UpdateCDDynamicsPlanner(tarJointState);
		eCtrlMode = CTRL_MODE_JOINT_POSITION;
		res.response = "OK";
	}
	else if(req.cmd.compare("ctrl_cmd") == 0)
	{
		ROS_INFO("Received request of updating controller parms...\n");
		ObjImpCtrlCmdHandler(req.parms);
		res.response = "OK";
	}
	else
	{
		ROS_INFO("Invalid request received.\n");
		res.response = "FAIL";
	}
	return true;
}

int main(int argc, char** argv)
{   
	//initialize node and check whether roscore is there
	int nRosArgc = 0;
	char** pRosArgv = NULL;
	ros::init(nRosArgc, pRosArgv, "ObjImpCtrlNode", ros::init_options::NoSigintHandler);

	if(!ros::master::check())
	{
		printf("Roscore is not started yet...\n");
		return 1;
	}

	//initialize impedance controller
	InitializeImpController(argc, argv);
	if(pImpCtrl == NULL)
	{
		return 1;
	}

	//initialize CDDynamics for generating release and grasp motion...
	pHandCDDyn = new CDDynamics(HAND_DOF_NUM, 0.001, 0.17);

	//initialize queue for contact point
	Vector3 zeroDummy;
	zeroDummy.Zero();
	for(int ind = 0; ind < nQueFilterLen; ++ind)
	{
		contactPosQue[0].push(zeroDummy);
		contactPosQue[1].push(zeroDummy);
		contactPosQue[2].push(zeroDummy);
		contactForceQue[0].push(zeroDummy);
		contactForceQue[1].push(zeroDummy);
		contactForceQue[2].push(zeroDummy);
	}
	//initialize subscriber/publisher, server as well as controller
	nh = new ros::NodeHandle();

	ros::Rate r(30);

	handStateSub = nh->subscribe(HAND_JOINT_STATE_TOPIC, 5, &HandJointStateCallback);
	handCmdPub = nh->advertise< sensor_msgs::JointState >(HAND_JOINT_CMD_TOPIC, 5);
	objCtrlCmdSub = nh->subscribe(OBJ_IMPCTRL_CMD_TOPIC, 5, &ObjImpCtrlCmdCallback);

	objPosePub = nh->advertise< geometry_msgs::PoseStamped >(OBJ_IMPCTRL_OBJ_POSE_STATE_TOPIC, 5);
	objPosePubNoTac = nh->advertise< geometry_msgs::PoseStamped >(OBJ_IMPCTRL_OBJ_POSE_STATE_TOPIC_NoTac, 5);

	contactPntF1Sub = nh->subscribe(CONTACT1_TOPIC, 5, &ContactF1Callback);
	contactPntF2Sub = nh->subscribe(CONTACT2_TOPIC, 5, &ContactF2Callback);
	contactPntF3Sub = nh->subscribe(CONTACT3_TOPIC, 5, &ContactF3Callback);

	ContactF1 =nh->advertise< geometry_msgs::PointStamped >("/objimpctrl/finger1_state", 5);
	ContactF2 =nh->advertise< geometry_msgs::PointStamped >("/objimpctrl/finger2_state", 5);
	ContactF3 =nh->advertise< geometry_msgs::PointStamped >("/objimpctrl/finger3_state", 5);


	//server
	nodeCmdSrv = nh->advertiseService(OBJ_IMPCTRL_CMD_SRV, ObjImpCtrlRequestHandler);

	while(ros::ok())
	{
		ros::spinOnce();

		switch(eCtrlMode)
		{
		case CTRL_MODE_JOINT_POSITION:
			//drive to target position of CDDynamics if control mode is joint position
		{
			/*
			if(bJntInitialized)
			{
				Vector thumb_pos(4), index_pos(4), middle_pos(4);
				ExtractFingerJoints(index_pos, middle_pos, thumb_pos);
				pImpCtrl->SetFingerChainState(0, index_pos);
				pImpCtrl->SetFingerChainState(1, middle_pos);
				pImpCtrl->SetFingerChainState(2, thumb_pos);
				//cout << "Set finger chain state" << endl;
				//update contact point information with averaged contact point
				Vector3 dummyForce;
				dummyForce.Zero();
				for(int i = 0; i < 3; ++i)
				{
					pImpCtrl->UpdateFingerContactInfo(i, dummyForce, avgCntctPnt[i]);
				}

				pImpCtrl->Update();
			}
			 */

			if(pHandCDDyn != NULL && bCDDynInit)
			{
				sensor_msgs::JointState handCmd;
				Vector tmpHandCmd;
				pHandCDDyn->Update();
				pHandCDDyn->GetState(tmpHandCmd);
				for(int i = 0; i < tmpHandCmd.Size(); ++i)
				{
					handCmd.position.push_back(tmpHandCmd(i));
				}
				//ignore pinky finger for grasp
				/*
				handCmd.position[8] = currJointState.position[8];
				handCmd.position[9] = currJointState.position[9];
				handCmd.position[10] = currJointState.position[10];
				handCmd.position[11] = currJointState.position[11];
				 */
				handCmdPub.publish(handCmd);
			}
		}
		break;
		case CTRL_MODE_OBJECT_IMPEDANCE:
		{
#ifdef QUATERNION_SLERP
			//change desired orientation if quaternion slerp is active
			if(bQuatSlerpActive)
			{
				Vector desiredQuat;
				QuaternionSlerp(quat1, quat2, lambda, desiredQuat);
				Matrix3 desiredOrient;
				GetRotationMatrixFromQuaternion(desiredQuat, desiredOrient);
				ObjState desiredObjState;
				pImpCtrl->GetDesiredObjState(desiredObjState);
				desiredObjState.mObjPose.SetOrient(desiredOrient);
				lambda += 0.001;    //1000 cycles
				if(lambda > 1)
				{
					bQuatSlerpActive = false;
					lambda = 0.0;
				}
			}

#endif
			//update model with sensory feedback

			Vector thumb_pos(4), index_pos(4), middle_pos(4);
			ExtractFingerJoints(index_pos, middle_pos, thumb_pos);
			pImpCtrl->SetFingerChainState(0, index_pos);
			pImpCtrl->SetFingerChainState(1, middle_pos);
			pImpCtrl->SetFingerChainState(2, thumb_pos);

			//update contact point information with averaged contact point
			Vector3 dummyForce;
			dummyForce.Zero();
			for(int i = 0; i < 3; ++i)
			{
				pImpCtrl->UpdateFingerContactInfo(i, dummyForce, avgCntctPnt[i]);
			}

			pImpCtrl->Update();

			//send out torque command from object impedance controller
			Vector computed_trqs;
			pImpCtrl->GetControlOutput(computed_trqs);
			sensor_msgs::JointState trqCmd;
			SerializeFingerTorquesToMsg(computed_trqs, trqCmd);
			handCmdPub.publish(trqCmd);
			break;
		}
		case CTRL_MODE_KINESTHETIC_TEACHING:
			//do nothing since zero torque was sent in handler
			break;
		default:
			//do nothing
			break;
		}

		//get current hand state and virtual obj frame
		geometry_msgs::PoseStamped poseMsg;
		ObjState objState;

		pImpCtrl->GetObjCurrentState(objState);
		ReferenceFrame baseFrame, objFrameWorld;
		//pImpCtrl->GetBaseRefFrame(baseFrame);
		//objFrameWorld.Set(baseFrame.Mult(objState.mObjPose));
		objFrameWorld.Set(objState.mObjPose);

		Vector3 objPos(objFrameWorld.GetOrigin());
		Matrix3 objOri(objFrameWorld.GetOrient());

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

//////////////////////////////////////////////// Miao -------------------
		//publish the VF using finger kinematics
		//get current hand state and virtual obj frame
		geometry_msgs::PoseStamped poseMsgNoTac;
		ObjState objStateNoTac;

		pImpCtrl->GetObjCurrentStateNoTac(objStateNoTac);
		ReferenceFrame baseFrameNoTac, objFrameWorldNoTac;
		//pImpCtrl->GetBaseRefFrame(baseFrame);
		//objFrameWorld.Set(baseFrame.Mult(objState.mObjPose));
		objFrameWorldNoTac.Set(objStateNoTac.mObjPose);

		Vector3 objPosNoTac(objFrameWorldNoTac.GetOrigin());
		Matrix3 objOriNoTac(objFrameWorldNoTac.GetOrient());

		Vector objOriQuatNoTac;
		objOriNoTac.GetQuaternionRepresentation(objOriQuatNoTac);
		poseMsgNoTac.pose.position.x = objPosNoTac(0);
		poseMsgNoTac.pose.position.y = objPosNoTac(1);
		poseMsgNoTac.pose.position.z = objPosNoTac(2);

		poseMsgNoTac.pose.orientation.w = objOriQuatNoTac(0);
		poseMsgNoTac.pose.orientation.x = objOriQuatNoTac(1);
		poseMsgNoTac.pose.orientation.y = objOriQuatNoTac(2);
		poseMsgNoTac.pose.orientation.z = objOriQuatNoTac(3);

		//time stamp
		poseMsgNoTac.header.stamp = ros::Time::now();
		objPosePubNoTac.publish(poseMsgNoTac);
/////////////////////////////////////////////////////////////////////////

		//publish finger state  --Miao
		geometry_msgs::PointStamped fingerMsg;
		MathLib::ReferenceFrame contactFrm;
		pImpCtrl->GetFingerContactPntFrm(0, contactFrm);
		Vector3 fingercon(contactFrm.GetOrigin());
		fingerMsg.point.x=fingercon(0);
		fingerMsg.point.y=fingercon(1);
		fingerMsg.point.z=fingercon(2);
		fingerMsg.header.stamp= ros::Time::now();
		ContactF1.publish(fingerMsg);

		pImpCtrl->GetFingerContactPntFrm(1, contactFrm);
		fingercon=contactFrm.GetOrigin();
		fingerMsg.point.x=fingercon(0);
		fingerMsg.point.y=fingercon(1);
		fingerMsg.point.z=fingercon(2);
		fingerMsg.header.stamp= ros::Time::now();
		ContactF2.publish(fingerMsg);

		pImpCtrl->GetFingerContactPntFrm(2, contactFrm);
		fingercon=contactFrm.GetOrigin();
		fingerMsg.point.x=fingercon(0);
		fingerMsg.point.y=fingercon(1);
		fingerMsg.point.z=fingercon(2);
		fingerMsg.header.stamp= ros::Time::now();
		ContactF3.publish(fingerMsg);



		//


		//r.sleep();
	}

	//release node
	nh->shutdown();
	delete nh;
	nh = NULL;

	if(pHandCDDyn != NULL)
	{
		delete pHandCDDyn;
		pHandCDDyn = NULL;
	}

	return 0;
}
