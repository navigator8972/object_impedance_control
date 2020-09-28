import os
import numpy as np

import gym
from gym import error, spaces, utils
from gym.utils import seeding

import pybullet as p
import pybullet_data, pybullet_envs


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
        self.observation = [0]*(7*self.n_arms)
        box_position = np.zeros(3).astype(np.float32)
        done = False
        reward = 0
        info = {'box_position':box_position}
        p.stepSimulation()
        return np.array(self.observation).astype(np.float32), reward, done, info

    def reset(self):
        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) # we will enable rendering after we loaded everything
        urdfRootPath=pybullet_data.getDataPath()
        p.setGravity(0,0,-10)

        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,0])

        rest_poses = [0, 0, 0, 0, 0, 0, 0]
        radius = 1.0
        self.arm1Uid = p.loadURDF(os.path.join(urdfRootPath, "kuka_iiwa/model.urdf"),useFixedBase=True, basePosition=[radius,0,0])
        self.arm2Uid = p.loadURDF(os.path.join(urdfRootPath, "kuka_iiwa/model.urdf"),useFixedBase=True, basePosition=[-radius*0.5,radius*0.866,0])
        self.arm3Uid = p.loadURDF(os.path.join(urdfRootPath, "kuka_iiwa/model.urdf"),useFixedBase=True, basePosition=[-radius*0.5,-radius*0.866,0])
        for i in range(7):
            p.resetJointState(self.arm1Uid,i, rest_poses[i])
            p.resetJointState(self.arm2Uid,i, rest_poses[i])
            p.resetJointState(self.arm3Uid,i, rest_poses[i])

        #create a base
        baseUid = p.loadURDF(os.path.join(urdfRootPath, "table_square/table_square.urdf"),useFixedBase=True)

        #create a box
        state_object= [np.random.uniform(-0.1,0.1),np.random.uniform(-0.1,0.1),1.0]
        half_ext = 0.35
        cuid = p.createCollisionShape(p.GEOM_BOX, halfExtents = [half_ext]*3)
        vuid = p.createVisualShape(p.GEOM_BOX, halfExtents = [half_ext]*3, rgbaColor=[0, 0, 1, 0.8])
        mass_box = 3.0
        self.objectUid = p.createMultiBody(mass_box,cuid,vuid, basePosition=state_object)

        self.observation = [0]*(7*self.n_arms)
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