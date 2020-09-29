from gym.envs.registration import register

register(
    id='multiarmbox-v0',
    entry_point='object_impedance_control.envs:MultiArmBoxEnv',
)