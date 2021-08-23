import os
import numpy as np

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
import pybullet_data, pybullet_envs
import pybullet_utils.bullet_client as bclient
import pybullet_utils.transformations as trans

from object_impedance_control.utils.init_utils import load_rigid_object, load_deform_object_mss, load_deform_object_nhk

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
        #allow deformables
        self.sim.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

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
        self.sim.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.sim.setGravity(self._gravity[0], self._gravity[1], self._gravity[2])

        planeUid = self.sim.loadURDF("plane.urdf", basePosition=[0,0,0])
        
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
                uid = self.sim.loadURDF("sphere_small.urdf",useFixedBase=False, basePosition=pos)
            else:
                ori = trans.quaternion_about_axis(angle, [0, 0, 1])
                ori = trans.quaternion_multiply(ori, trans.quaternion_about_axis(np.pi, [0, 0, 1]))
                uid = self.sim.loadURDF("kuka_iiwa/model.urdf",useFixedBase=True, basePosition=pos, baseOrientation=ori)
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
        baseUid = self.sim.loadURDF("table_square/table_square.urdf",useFixedBase=True)

        #create an object
        # state_object= [np.random.uniform(-0.1,0.1)
        #     ,np.random.uniform(-0.1,0.1)
        #     ,1.0] + list(trans.quaternion_from_euler(np.pi/2, 0, 0, axes='sxyz'))
        init_pos = self._args.object_init_pos
        init_rot = self._args.object_init_rot
        state_object= init_pos + list(trans.quaternion_from_euler(init_rot[0], init_rot[1], init_rot[2], axes='sxyz'))
        if self._args.object_deform:
            # self.objectUid = load_deform_object_mss(
            #         self.sim, 
            #         os.path.join(urdfRootPath, self._args.object_file), 
            #         None,
            #         self._args.object_scale, 
            #         self._args.object_mass, 
            #         state_object[:3], 
            #         state_object[3:], 
            #         10,     # bending_stiffness, 
            #         0.01,   # damping_stiffness, 
            #         100,    # elastic_stiffness,
            #         0.5,    # friction 
            #         0       # debug flag
            #         )
            self.objectUid = load_deform_object_nhk(
                self.sim, 
                self._args.object_file, 
                None,
                self._args.object_scale, 
                self._args.object_mass, 
                state_object[:3], 
                state_object[3:], 
                180,    # neohookean mu
                60,    # neohookean lambda
                0.01,   # neohookean damping
                3.0,    # friction 
                0       # debug flag
                )
        else:
            self.objectUid = load_rigid_object(
                    self.sim, 
                    self._args.object_file, 
                    self._args.object_scale, 
                    self._args.object_mass, 
                    state_object[:3], state_object[3:], 
                    texture_file=None,
                    # rgba_color=[0, 0, 1, 0.8]
                    )
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

from object_impedance_control.controllers.object_impedance_controller import MultiAgentObjectImpedanceController

class MultiAgentObjectImpedanceControlEnv(MultiAgentObjectEnv):
    def __init__(self, args):
        super().__init__(args)

        #prepare an object level impedance controller on top of the environment: 
        # the user access to params of the controller instead of the low-level robot control
        self.controller = MultiAgentObjectImpedanceController(self.n_agents, 120, 10, agent_link_stiffness=30)
        self.controller.set_desired_object_pos(np.array([0, 0, 1.1]))

        #override the action and state space specification
        #observation is the position of endeffectors/agents
        self.observation_space = spaces.Box(np.array([-1]*(3*self.n_agents)), np.array([1]*(3*self.n_agents)))
        #action space is populated by controller parameters
        low = np.concatenate([
            -np.ones(3),                                #desired object position
            np.zeros(3),                                #desired object orientation. euler angle sxyz format, this might be preferred over quaternion for the convenicen of euclidean?
            np.ones(3)*1e-3,                            #commanded object translational impedance, only diagonal part
            np.ones(3)*1e-3,                            #commanded object rotational impedance, only diagonal part
            np.ones(3*self.n_agents)*1e-3               #agent link stiffness, only diagonal part in the tip local frame of reference
        ])

        high = np.concatenate([
            np.ones(3),                                 #desired object position
            np.array([np.pi*2, np.pi, np.pi*2]),        #desired object orientation. euler angle sxyz format, this might be preferred over quaternion for the convenicen of euclidean?
            np.ones(3)*300,                             #commanded object translational impedance, only diagonal part
            np.ones(3)*100,                             #commanded object rotational impedance, only diagonal part
            np.ones(3*self.n_agents)*100                #agent link stiffness, only diagonal part in the tip local frame of reference
        ])
        self.action_space = spaces.Box(low, high)
    

    def get_obs(self):
        agent_pos = super().get_obs()
        if self._args.agent_model == 'anchor':
            return agent_pos
        else:
            #note for objimpcontroller, the observation only cares about tip position
            return np.concatenate(self.get_forward_kinematics_pos())

    
    def step(self, action):
        if self._args.viz:
            self.sim.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        
        obs = self.get_obs()

        obj_position = self.sim.getBasePositionAndOrientation(self.objectUid)[:3]
        done = False
        reward = 0
        info = {'obj_position':obj_position}

        #process action as controller parameters
        params = self.get_params_from_action(action)

        self.controller.set_desired_object_pos(params['desired_object_pos'])
        self.controller.set_desired_object_rot(params['desired_object_rot'])
        self.controller.set_object_stiffness(params['object_stiffness_trans'], params['object_stiffness_rot'])
        for i in range(self.n_agents):
            self.controller.set_agent_link_stiffness(i, params['agent_link_stiffness'][i])

        act_forces = self.controller.step(obs.reshape((self.n_agents, -1)), agent_force=None)

        if self._args.agent_model != 'anchor':
            act_forces = np.array(self.get_arm_action(act_forces))

        act_forces = act_forces.flatten()

        #apply gravity compensation
        for i, id in enumerate(self.agentUIDs):
            if self._args.agent_model == 'anchor':
                agent_gravity = self._gravity * self.sim.getDynamicsInfo(id, -1)[0]
                damping = 15.0
                agent_vel, _ = self.sim.getBaseVelocity(id)
                effect = act_forces[i*3:(i+1)*3] - agent_gravity - np.array(agent_vel) * damping
                self.sim.applyExternalForce(id, -1, effect, self.agent_pos[i], self.sim.WORLD_FRAME)
            else:
                zero_vec = [0.0]*self.nJointsPerArm
                #note this calculated desired gravity to realized dynamical params
                agent_gravity = self.sim.calculateInverseDynamics(id, self.agent_pos[i], zero_vec, zero_vec)
                effect = act_forces[i*self.nJointsPerArm:(i+1)*self.nJointsPerArm] + agent_gravity
                self.sim.setJointMotorControlArray(id, range(self.nJointsPerArm), self.sim.TORQUE_CONTROL, forces=effect)
     
        self.sim.stepSimulation()
        return obs.astype(np.float32), reward, done, info 
    
    def get_params_from_action(self, action):
        #parse action vector to a dict with more interpretable keys
        ctrl_params = {
            'desired_object_pos':       action[:3],
            'desired_object_rot':       trans.quaternion_from_euler(action[3], action[4], action[5], axes='sxyz'),
            'object_stiffness_trans':   np.diag(action[6:9]),
            'object_stiffness_rot':     np.diag(action[9:12]),
            'agent_link_stiffness':     [np.diag(action[(12+i*3):(12+(i+1)*3)]) for i in range(self.n_agents)]
        }
        return ctrl_params
    
    def get_action_from_params(self, params):
        action = np.concatenate([
            params['desired_object_pos'],
            trans.euler_from_quaternion(params['desired_object_rot'], axes='sxyz'),
            np.diag(params['object_stiffness_trans']),
            np.diag(params['object_stiffness_rot']),
            np.concatenate([np.diag(params['agent_link_stiffness'][i]) for i in range(self.n_agents)])
        ])
        return action
    
    def get_controller_params(self):
        #not the controller stiffness parameters might not be diagonal matrices
        params = {
            'desired_object_pos':       self.controller._desired_object_pos,
            'desired_object_rot':       self.controller._desired_object_rot,
            'object_stiffness_trans':   self.controller._object_stiffness_trans,
            'object_stiffness_rot':     self.controller._object_stiffness_rot,
            'agent_link_stiffness':     self.controller._agent_link_stiffness
        }
        return params