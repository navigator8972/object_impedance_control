from time import sleep

import numpy as np

import gym

from object_impedance_control.controllers.object_impedance_controller import MultiAgentObjectImpedanceController

def main():
    """
    Tests importing of gym envs
    """
    env = gym.make('multiarmbox-v0')

    env.reset()

    controller = MultiAgentObjectImpedanceController(3, 80, 0, agent_link_stiffness=20)
    controller.set_desired_object_pos(np.array([0, 0, 1.2]))

    while True:
        ee_states = env.get_forward_kinematics_pos()
        agent_pos = np.array(ee_states)
        act_forces = controller.step(agent_pos, agent_force=None)
        # print(act_forces)
        arm1_jac, arm2_jac, arm3_jac = env.get_position_jacobian()

        arm1_jnt_trq = act_forces[0].dot(np.array(arm1_jac))
        arm2_jnt_trq = act_forces[1].dot(np.array(arm2_jac))
        arm3_jnt_trq = act_forces[2].dot(np.array(arm3_jac))

        env.step(np.array(np.concatenate([arm1_jnt_trq, arm2_jnt_trq, arm3_jnt_trq])))
        # env.step(np.zeros(21))
        # env.render()

        sleep(1./240.)


    return True


def test_import():
    assert main() is True


if __name__ == '__main__':
    main()