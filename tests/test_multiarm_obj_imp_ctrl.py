import numpy as np

import gym

import object_impedance_control

def main():
    """
    Tests importing of gym envs
    """
    env = gym.make('multiarmbox-v0')

    env.reset()

    while True:
        env.step(np.random.rand(7*3))
        env.render()

    return True


def test_import():
    assert main() is True


if __name__ == '__main__':
    main()