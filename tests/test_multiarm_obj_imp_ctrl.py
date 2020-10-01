from time import sleep

import numpy as np

import gym

from object_impedance_control.controllers.object_impedance_controller import MultiAgentObjectImpedanceController
import object_impedance_control.controllers.utils as utils

def main():
    """
    Tests importing of gym envs
    """
    env = gym.make('multiarmbox-v0')

    env.reset()

    controller = MultiAgentObjectImpedanceController(3, 120, 10, agent_link_stiffness=30)
    controller.set_desired_object_pos(np.array([0, 0, 1.2]))

    while True:
        ee_states = env.get_forward_kinematics_pos()
        agent_pos = np.array(ee_states)
        act_forces = controller.step(agent_pos, agent_force=None)

        pos1, rot1, pos2, rot2, pos3, rot3 = env.get_arm_base_pose()
        # get forces in the robot base reference frame
        act_forces = np.array([
            act_forces[0].dot(utils.quaternion_to_matrix(rot1)),
            act_forces[1].dot(utils.quaternion_to_matrix(rot2)),
            act_forces[2].dot(utils.quaternion_to_matrix(rot3))
        ])

        arm1_jac, arm2_jac, arm3_jac = env.get_position_jacobian()

        arm1_jnt_trq = act_forces[0].dot(np.array(arm1_jac))
        arm2_jnt_trq = act_forces[1].dot(np.array(arm2_jac))
        arm3_jnt_trq = act_forces[2].dot(np.array(arm3_jac))

        env.step(np.array(np.concatenate([arm1_jnt_trq, arm2_jnt_trq, arm3_jnt_trq])))

        sleep(1./240.)


    return True


if __name__ == '__main__':
    main()