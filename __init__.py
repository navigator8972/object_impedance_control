from gym.envs.registration import register

register(
    id='multiarmbox-v0',
    entry_point='object_impedance_control.envs:MultiArmBoxEnv',
)

register(
    id='multiagentbox-v0',
    entry_point='object_impedance_control.envs:MultiAgentBoxEnv',
)

register(
    id='multiagentobj-v0',
    entry_point='object_impedance_control.envs:MultiAgentObjectEnv',
)