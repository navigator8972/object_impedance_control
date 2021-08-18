from time import sleep

import numpy as np

import gym

from object_impedance_control.controllers.object_impedance_controller import MultiAgentObjectImpedanceController
from object_impedance_control.utils.args_utils import get_args

def main_multiagent(args):
    kwargs = {'args': args}
    # env = gym.make('multiagentbox-v0')
    env = gym.make('multiagentobj-v0', **kwargs)
    # env.seed(env._args.seed)
    env.reset()

    controller = MultiAgentObjectImpedanceController(3, 80, 10, agent_link_stiffness=20)
    controller.set_desired_object_pos(np.array([0, 0, 1.2]))

    while True:
        if args[0].agent_model == 'anchor':
            state = env._get_state()
            agent_pos = np.reshape(state, (3, -1))[:, :3]
        else:
            ee_states = env.get_forward_kinematics_pos()
            agent_pos = np.array(ee_states)
        act_forces = controller.step(agent_pos, agent_force=None)
        
        if args[0].agent_model != 'anchor':
            act_forces = np.array(env.get_arm_action(act_forces))

        env.step(act_forces.flatten())

        sleep(1./240.)


    return True

import copy

def main_multiagentobjimpctrl(args):
    kwargs = {'args': args}
    env = gym.make('multiagentobjimpctrl-v0', **kwargs)
    # env.seed(env._args.seed)
    env.reset()
    init_ctrl_params = env.get_controller_params()
    dt = 1./240.
    i = 0
    traj_start = 240
    while True:
        ctrl_params = copy.deepcopy(init_ctrl_params)
        if i > traj_start:
            #apply a trajectory after 1s
            ctrl_params['desired_object_pos'][2] += np.sin(2*np.pi*(i-traj_start)*dt) * 0.2
        action = env.get_action_from_params(ctrl_params)
        env.step(action)
        sleep(dt)
        i+=1

    return True

if __name__ == '__main__':
    # main_multiagent(get_args())
    main_multiagentobjimpctrl(get_args())