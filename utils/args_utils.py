import argparse
import logging
import sys

def get_args():
    logging.basicConfig(
        level=logging.INFO, format='%(asctime)s %(message)s',
        handlers=[logging.StreamHandler(sys.stdout)])
    parser = argparse.ArgumentParser(description='args', add_help=True)
    # Main/demo args.
    parser.add_argument('--object_file', type=str,
                        default='cube.obj', help='Object mesh file. Default at pybullet_data folder.')
    parser.add_argument('--object_scale', type=float,
                        default=0.75, help='Object geometry scale.')
    parser.add_argument('--object_mass', type=float,
                        default=0.5, help='Object mass.')
    parser.add_argument('--agent_model', type=str,
                        choices=['anchor', 'kuka'], default='anchor', help='Model of agents for holding the object')
    parser.add_argument('--agent_num', type=int,
                        choices=[3, 4], default=3, help='Number of agents for holding the object')
    parser.add_argument('--seed', type=int, default=0, help='Random seed')
    parser.add_argument('--viz', action='store_true', help='Enable visualization.')
    parser.add_argument('--debug', action='store_true',
                        help='Verbose for debug information.')
    # Simulation args. Note: turn up frequency when deform stiffness is high.
    parser.add_argument('--sim_gravity', type=float, default=-9.8,
                        help='Gravity constant for PyBullet simulation.')

    # Parse args and do sanity checks.
    args = parser.parse_known_args()
    return args