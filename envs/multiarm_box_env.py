import os
import numpy as np

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
import pybullet_data, pybullet_envs

#method to filter out unmotored joints
def getMotorJointStates(robot):
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
    joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques

class MultiArmBoxEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.step_counter = 0
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])
        
        self.n_arms = 3
        self.action_space = spaces.Box(np.array([-1]*(7*self.n_arms)), np.array([1]*(7*self.n_arms)))
        #for now only observe joint positions, can extend to other sensor modalities?
        self.observation_space = spaces.Box(np.array([-1]*(7*self.n_arms)), np.array([1]*(7*self.n_arms)))

    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        arm1_joint_state = getMotorJointStates(self.arm1Uid)
        arm2_joint_state = getMotorJointStates(self.arm2Uid)
        arm3_joint_state = getMotorJointStates(self.arm3Uid)

        arm1_joint_pos = arm1_joint_state[0]
        arm2_joint_pos = arm2_joint_state[0]
        arm3_joint_pos = arm3_joint_state[0]

        self.observation = np.concatenate((arm1_joint_pos, arm2_joint_pos, arm3_joint_pos))

        box_position = p.getBasePositionAndOrientation(self.objectUid)[:3]
        done = False
        reward = 0
        info = {'box_position':box_position}

        #apply gravity compensation
        zero_vec = [0.0]*self.nJointsPerArm
        arm1_gravity = p.calculateInverseDynamics(self.arm1Uid, arm1_joint_pos, zero_vec, zero_vec)
        arm2_gravity = p.calculateInverseDynamics(self.arm1Uid, arm2_joint_pos, zero_vec, zero_vec)
        arm3_gravity = p.calculateInverseDynamics(self.arm1Uid, arm3_joint_pos, zero_vec, zero_vec)

        #apply damping term, joints have some default dampings... and this additional term makes system unstable, why??
        damping = 0.0

        arm1_trq = action[:self.nJointsPerArm] + arm1_gravity - np.array(arm1_joint_state[1]) * damping
        arm2_trq = action[self.nJointsPerArm:2*self.nJointsPerArm] + arm2_gravity - np.array(arm2_joint_state[1]) * damping
        arm3_trq = action[2*self.nJointsPerArm:3*self.nJointsPerArm] + arm3_gravity - np.array(arm3_joint_state[1]) * damping
       
        
        #apply action
        p.setJointMotorControlArray(self.arm1Uid, range(7), p.TORQUE_CONTROL, forces=arm1_trq)
        p.setJointMotorControlArray(self.arm2Uid, range(7), p.TORQUE_CONTROL, forces=arm2_trq)
        p.setJointMotorControlArray(self.arm3Uid, range(7), p.TORQUE_CONTROL, forces=arm3_trq)

        p.stepSimulation()
        return np.array(self.observation).astype(np.float32), reward, done, info
    
    def get_forward_kinematics_pos(self):
        #return end-effector positions
        arm1_ee_state = p.getLinkState(self.arm1Uid,
                        self.nJointsPerArm-1,
                        computeLinkVelocity=0,
                        computeForwardKinematics=0)
        arm2_ee_state = p.getLinkState(self.arm2Uid,
                        self.nJointsPerArm-1,
                        computeLinkVelocity=0,
                        computeForwardKinematics=0)
        arm3_ee_state = p.getLinkState(self.arm3Uid,
                        self.nJointsPerArm-1,
                        computeLinkVelocity=0,
                        computeForwardKinematics=0)
        return arm1_ee_state[0], arm2_ee_state[0], arm3_ee_state[0]
    
    def get_position_jacobian(self):
        #position jacobian w.r.t the joint position at end-effector link
        #note there would be an offset to the real contact point
        zero_vec = [0.0]*self.nJointsPerArm
        com_pos = [0.0]*3
        com_pos[2] = 0.02
        #warning: we cannot directly feed numpy array to calculateJacobian. it dumps a segment fault...
        obs_lst = list(self.observation)
        jac_t_1, jac_r_1 = p.calculateJacobian(self.arm1Uid, self.nJointsPerArm-1, com_pos, obs_lst[:7], zero_vec, zero_vec)
        jac_t_2, jac_r_2 = p.calculateJacobian(self.arm2Uid, self.nJointsPerArm-1, com_pos, obs_lst[7:14], zero_vec, zero_vec)
        jac_t_3, jac_r_3 = p.calculateJacobian(self.arm3Uid, self.nJointsPerArm-1, com_pos, obs_lst[14:21], zero_vec, zero_vec)
        return jac_t_1, jac_t_2, jac_t_3

    def reset(self):
        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
        urdfRootPath=pybullet_data.getDataPath()
        p.setGravity(0,0,-10)

        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,0])

        rest_poses = [0, -0.2, 0, -1.2, 0, 0, 0]
        radius = 0.8
        self.arm1Uid = p.loadURDF(os.path.join(urdfRootPath, "kuka_iiwa/model.urdf"),useFixedBase=True, basePosition=[radius,0,0], baseOrientation=[ 0, 0, 1, 0 ])
        self.arm2Uid = p.loadURDF(os.path.join(urdfRootPath, "kuka_iiwa/model.urdf"),useFixedBase=True, basePosition=[-radius*0.5,radius*0.866,0], baseOrientation=[ 0, 0, 0.5, -0.8660254 ])
        self.arm3Uid = p.loadURDF(os.path.join(urdfRootPath, "kuka_iiwa/model.urdf"),useFixedBase=True, basePosition=[-radius*0.5,-radius*0.866,0], baseOrientation=[ 0, 0, -0.5, -0.8660254 ])
        
        self.nJointsPerArm = p.getNumJoints(self.arm1Uid)
        
        for i in range(7):
            p.resetJointState(self.arm1Uid,i, rest_poses[i])
            p.resetJointState(self.arm2Uid,i, rest_poses[i])
            p.resetJointState(self.arm3Uid,i, rest_poses[i])

            #we need to first set force limit to zero to use torque control!!
            #see: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_dynamics.py
            p.setJointMotorControlArray(self.arm1Uid, range(self.nJointsPerArm), p.VELOCITY_CONTROL, forces=np.zeros(self.nJointsPerArm))
            p.setJointMotorControlArray(self.arm2Uid, range(self.nJointsPerArm), p.VELOCITY_CONTROL, forces=np.zeros(self.nJointsPerArm))
            p.setJointMotorControlArray(self.arm3Uid, range(self.nJointsPerArm), p.VELOCITY_CONTROL, forces=np.zeros(self.nJointsPerArm))

            # p.setJointMotorControlArray(self.arm1Uid, range(self.nJointsPerArm), p.TORQUE_CONTROL, forces=np.zeros(self.nJointsPerArm))
            # p.setJointMotorControlArray(self.arm2Uid, range(self.nJointsPerArm), p.TORQUE_CONTROL, forces=np.zeros(self.nJointsPerArm))
            # p.setJointMotorControlArray(self.arm3Uid, range(self.nJointsPerArm), p.TORQUE_CONTROL, forces=np.zeros(self.nJointsPerArm))

        #create a base
        baseUid = p.loadURDF(os.path.join(urdfRootPath, "table_square/table_square.urdf"),useFixedBase=True)

        #create a box
        state_object= [np.random.uniform(-0.1,0.1),np.random.uniform(-0.1,0.1),1.0]
        half_ext = 0.35
        cuid = p.createCollisionShape(p.GEOM_BOX, halfExtents = [half_ext]*3)
        vuid = p.createVisualShape(p.GEOM_BOX, halfExtents = [half_ext]*3, rgbaColor=[0, 0, 1, 0.8])
        mass_box = 0.5
        self.objectUid = p.createMultiBody(mass_box,cuid,vuid, basePosition=state_object)

        #get observations, only joint positions are considered
        arm1_joint_state = getMotorJointStates(self.arm1Uid)
        arm2_joint_state = getMotorJointStates(self.arm2Uid)
        arm3_joint_state = getMotorJointStates(self.arm3Uid)

        arm1_joint_pos = arm1_joint_state[0]
        arm2_joint_pos = arm2_joint_state[0]
        arm3_joint_pos = arm3_joint_state[0]

        self.observation = np.concatenate((arm1_joint_pos, arm2_joint_pos, arm3_joint_pos))
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        return np.array(self.observation).astype(np.float32)

    def render(self, mode='human'):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=90,
                                                            pitch=-70,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def _get_state(self):
        return self.observation

    def close(self):
        p.disconnect()