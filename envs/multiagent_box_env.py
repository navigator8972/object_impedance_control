import os
import numpy as np

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
import pybullet_data, pybullet_envs


class MultiAgentBoxEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.step_counter = 0
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])
        
        self.n_agents = 3
        self.action_space = spaces.Box(np.array([-1]*(7*self.n_agents)), np.array([1]*(7*self.n_agents)))
        #for now only observe joint positions, can extend to other sensor modalities?
        self.observation_space = spaces.Box(np.array([-1]*(7*self.n_agents)), np.array([1]*(7*self.n_agents)))

    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        agent1_pose = np.concatenate(p.getBasePositionAndOrientation(self.agent1Uid))
        agent2_pose = np.concatenate(p.getBasePositionAndOrientation(self.agent2Uid))
        agent3_pose = np.concatenate(p.getBasePositionAndOrientation(self.agent3Uid))

        self.observation = np.concatenate([agent1_pose, agent2_pose, agent3_pose])

        box_position = p.getBasePositionAndOrientation(self.objectUid)[:3]
        done = False
        reward = 0
        info = {'box_position':box_position}

        #gravity compensation
        action[:3] = action[:3] + np.array([0, 0, 10]) * p.getDynamicsInfo(self.agent1Uid, -1)[0]
        action[3:6] = action[3:6] + np.array([0, 0, 10]) * p.getDynamicsInfo(self.agent2Uid, -1)[0]
        action[6:9] = action[6:9] + np.array([0, 0, 10]) * p.getDynamicsInfo(self.agent3Uid, -1)[0]

        #damping, only consider linear vel
        damping = 15.0
        agent1_vel, _ = p.getBaseVelocity(self.agent1Uid)
        agent2_vel, _ = p.getBaseVelocity(self.agent2Uid)
        agent3_vel, _ = p.getBaseVelocity(self.agent3Uid)

        action[:3] = action[:3] - np.array(agent1_vel) * damping
        action[3:6] = action[3:6] - np.array(agent2_vel) * damping
        action[6:9] = action[6:9] - np.array(agent3_vel) * damping

        #apply force to each agent
        p.applyExternalForce(self.agent1Uid, -1, action[:3], agent1_pose[:3], p.WORLD_FRAME)
        p.applyExternalForce(self.agent2Uid, -1, action[3:6], agent2_pose[:3], p.WORLD_FRAME)
        p.applyExternalForce(self.agent3Uid, -1, action[6:9], agent3_pose[:3], p.WORLD_FRAME)

        p.stepSimulation()
        return np.array(self.observation).astype(np.float32), reward, done, info

    def reset(self):
        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
        p.resetDebugVisualizerCamera( cameraDistance=3, cameraYaw=30, cameraPitch=-32, cameraTargetPosition=[0,0,0])

        urdfRootPath=pybullet_data.getDataPath()
        p.setGravity(0,0,-10)

        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,0])

        rest_poses = [0, 0, 0, 0, 0, 0, 0]
        radius = 0.5
        self.agent1Uid = p.loadURDF(os.path.join(urdfRootPath, "sphere_small.urdf"),useFixedBase=False, basePosition=[radius,0,0.8])
        self.agent2Uid = p.loadURDF(os.path.join(urdfRootPath, "sphere_small.urdf"),useFixedBase=False, basePosition=[-radius*0.5,radius*0.866,0.8])
        self.agent3Uid = p.loadURDF(os.path.join(urdfRootPath, "sphere_small.urdf"),useFixedBase=False, basePosition=[-radius*0.5,-radius*0.866,0.8])

        p.changeDynamics(self.agent1Uid, -1, lateralFriction=2.0, spinningFriction=0.8, rollingFriction=0.8)
        p.changeDynamics(self.agent2Uid, -1, lateralFriction=2.0, spinningFriction=0.8, rollingFriction=0.8)
        p.changeDynamics(self.agent3Uid, -1, lateralFriction=2.0, spinningFriction=0.8, rollingFriction=0.8)

        #create a base
        baseUid = p.loadURDF(os.path.join(urdfRootPath, "table_square/table_square.urdf"),useFixedBase=True)

        #create a box
        state_object= [np.random.uniform(-0.1,0.1),np.random.uniform(-0.1,0.1),1.0]
        half_ext = 0.35
        cuid = p.createCollisionShape(p.GEOM_BOX, halfExtents = [half_ext]*3)
        vuid = p.createVisualShape(p.GEOM_BOX, halfExtents = [half_ext]*3, rgbaColor=[0, 0, 1, 0.8])
        mass_box = 0.5
        self.objectUid = p.createMultiBody(mass_box,cuid,vuid, basePosition=state_object)
        p.changeDynamics(self.objectUid, -1, lateralFriction=2.0, spinningFriction=0.8, rollingFriction=0.8)

        agent1_pose = np.concatenate(p.getBasePositionAndOrientation(self.agent1Uid))
        agent2_pose = np.concatenate(p.getBasePositionAndOrientation(self.agent2Uid))
        agent3_pose = np.concatenate(p.getBasePositionAndOrientation(self.agent3Uid))

        self.observation = np.concatenate([agent1_pose, agent2_pose, agent3_pose])
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