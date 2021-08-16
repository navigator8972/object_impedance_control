import os
import numpy as np

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
import pybullet_data, pybullet_envs
import pybullet_utils.bullet_client as bclient
import pybullet_utils.transformations as trans

from object_impedance_control.utils.init_utils import load_rigid_object

#method to filter out unmotored joints
def getMotorJointStates(bc, robot):
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
    joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques

class MultiAgentObjectEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, args):
        self.step_counter = 0

        self.sim = bclient.BulletClient(
            connection_mode=p.GUI if args[0].viz else p.DIRECT)

        self.sim.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])
        
        self._args = args[0]
        self._gravity = np.array([0, 0, args[0].sim_gravity])

        self.n_agents = args[0].agent_num
        if args[0].agent_model == 'anchor':
            #this is agent represented by sphere. rigid pose position
            self.action_space = spaces.Box(np.array([-1]*(3*self.n_agents)), np.array([1]*(3*self.n_agents)))
            self.observation_space = spaces.Box(np.array([-1]*(3*self.n_agents)), np.array([1]*(3*self.n_agents)))
        else:
            #this is kuka for now. we probably need to add more flexibility to parse actuatable dof from urdf
            self.action_space = spaces.Box(np.array([-1]*(7*self.n_agents)), np.array([1]*(7*self.n_agents)))
            #for now only observe joint positions, can extend to other sensor modalities?
            self.observation_space = spaces.Box(np.array([-1]*(7*self.n_agents)), np.array([1]*(7*self.n_agents)))

    def get_obs(self):
        if self._args.agent_model == 'anchor':
            self.agent_pos = [self.sim.getBasePositionAndOrientation(id)[0] for id in self.agentUIDs]
        else:
            self.agent_pos = [getMotorJointStates(self.sim, id)[0] for id in self.agentUIDs]
        obs = np.concatenate(self.agent_pos)
        # print(self.agent_pos, self._args.agent_model)
        return obs

    def step(self, action):
        if self._args.viz:
            self.sim.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        
        obs = self.get_obs()

        obj_position = self.sim.getBasePositionAndOrientation(self.objectUid)[:3]
        done = False
        reward = 0
        info = {'obj_position':obj_position}

        #apply gravity compensation
        for i, id in enumerate(self.agentUIDs):
            if self._args.agent_model == 'anchor':
                agent_gravity = self._gravity * self.sim.getDynamicsInfo(id, -1)[0]
                damping = 15.0
                agent_vel, _ = self.sim.getBaseVelocity(id)
                effect = action[i*3:(i+1)*3] - agent_gravity - np.array(agent_vel) * damping
                self.sim.applyExternalForce(id, -1, effect, self.agent_pos[i], self.sim.WORLD_FRAME)
            else:
                zero_vec = [0.0]*self.nJointsPerArm
                #note this calculated desired gravity to realized dynamical params
                agent_gravity = self.sim.calculateInverseDynamics(id, self.agent_pos[i], zero_vec, zero_vec)
                effect = action[i*self.nJointsPerArm:(i+1)*self.nJointsPerArm] + agent_gravity
                self.sim.setJointMotorControlArray(id, range(self.nJointsPerArm), self.sim.TORQUE_CONTROL, forces=effect)
     
        self.sim.stepSimulation()
        return obs.astype(np.float32), reward, done, info
    
    def get_forward_kinematics_pos(self):
        #return end-effector positions
        arm_ee_state = [self.sim.getLinkState(id,
                        self.nJointsPerArm-1,
                        computeLinkVelocity=0,
                        computeForwardKinematics=0)[0] for i, id in enumerate(self.agentUIDs)]

        return arm_ee_state
    
    def get_position_jacobian(self):
        #position jacobian w.r.t the joint position at end-effector link
        #note there would be an offset to the real contact point
        zero_vec = [0.0]*self.nJointsPerArm
        com_pos = [0.0]*3
        com_pos[2] = 0.02   #TODO: this is only valid for kuka
        #warning: we cannot directly feed numpy array to calculateJacobian. it dumps a segment fault...
        jac_t_lst = [self.sim.calculateJacobian(id, self.nJointsPerArm-1, com_pos, self.agent_pos[i], zero_vec, zero_vec)[0] for i, id in enumerate(self.agentUIDs)]
        return jac_t_lst

    def get_arm_base_pose(self):
        arm_base_pose = [self.sim.getBasePositionAndOrientation(id) for id in self.agentUIDs]
        return arm_base_pose
    
    def get_arm_action(self, act_forces):
        arm_bases = self.get_arm_base_pose()
        # get forces in the robot base reference frame
        local_forces = [act_forces[i].dot(trans.quaternion_matrix(arm_bases[i][1])[:3, :3]) for i in range(self.n_agents)]
        jacobians = self.get_position_jacobian()
        arm_torques = [f.dot(j) for f, j in zip(local_forces, jacobians)]
        return arm_torques

    def reset(self):
        self.step_counter = 0
        self.sim.resetSimulation()
        if self._args.viz:
            self.sim.configureDebugVisualizer(self.sim.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
            self.sim.resetDebugVisualizerCamera( cameraDistance=3, cameraYaw=30, cameraPitch=-32, cameraTargetPosition=[0,0,0])
        urdfRootPath=pybullet_data.getDataPath()
        self.sim.setGravity(self._gravity[0], self._gravity[1], self._gravity[2])

        planeUid = self.sim.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,0])
        
        if self._args.agent_model == 'anchor':
            rest_poses = [0, 0, 0, 0, 0, 0, 0]
            radius = 0.5
            self.nJointsPerArm = None
            base_height = 0.8
        else:
            rest_poses = [0, -0.2, 0, -1.2, 0, 0, 0]
            radius = 0.8
            base_height = 0.0

        
        self.agentUIDs = []
        angle = 0
        for i in range(self.n_agents):
            pos = [radius*np.cos(angle), radius*np.sin(angle), base_height]
            if self._args.agent_model == 'anchor':
                uid = self.sim.loadURDF(os.path.join(urdfRootPath, "sphere_small.urdf"),useFixedBase=False, basePosition=pos)
            else:
                ori = trans.quaternion_about_axis(angle, [0, 0, 1])
                ori = trans.quaternion_multiply(ori, trans.quaternion_about_axis(np.pi, [0, 0, 1]))
                uid = self.sim.loadURDF(os.path.join(urdfRootPath, "kuka_iiwa/model.urdf"),useFixedBase=True, basePosition=pos, baseOrientation=ori)
            angle += np.radians(360.0/self.n_agents)
            self.agentUIDs.append(uid)
            

        if self._args.agent_model != 'anchor':
            self.nJointsPerArm = self.sim.getNumJoints(self.agentUIDs[0])
            for id in self.agentUIDs:
                for i in range(self.nJointsPerArm):
                    self.sim.resetJointState(id,i, rest_poses[i])
                #we need to first set force limit to zero to use torque control!!
                #see: https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_dynamics.py
                self.sim.setJointMotorControlArray(id, range(self.nJointsPerArm), self.sim.VELOCITY_CONTROL, forces=np.zeros(self.nJointsPerArm))

        #create a base
        baseUid = self.sim.loadURDF(os.path.join(urdfRootPath, "table_square/table_square.urdf"),useFixedBase=True)

        #create an object
        state_object= [np.random.uniform(-0.1,0.1),np.random.uniform(-0.1,0.1),1.0]
        self.objectUid = load_rigid_object(
                self.sim, 
                os.path.join(urdfRootPath, self._args.object_file), 
                self._args.object_scale, 
                self._args.object_mass, 
                state_object[:3], state_object[3:], 
                texture_file=None,
                rgba_color=[0, 0, 1, 0.8])
        self.sim.changeDynamics(self.objectUid, -1, lateralFriction=3.0, spinningFriction=0.8, rollingFriction=0.8)

        obs = self.get_obs()
        if self._args.viz:
            self.sim.configureDebugVisualizer(self.sim.COV_ENABLE_RENDERING,1)
        return obs.astype(np.float32)

    def render(self, mode='human'):
        view_matrix = self.sim.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=90,
                                                            pitch=-70,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = self.sim.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (_, _, px, _, _) = self.sim.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def _get_state(self):
        return self.get_obs()

    def close(self):
        self.sim.disconnect()