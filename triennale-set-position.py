#!/usr/bin/env python3

import argparse
import triennale

def run(robot_id, position, velocity):
    print(f"Setting robot {robot_id} to position {position} at velocity {velocity}")
    robot = triennale.TriennaleRobot(robot_id)
    robot.set_position(position, velocity)


def main():
    parser = argparse.ArgumentParser(
        description='Set position and velocity for a single robot_id: A, B, C, or D.'
    )
    parser.add_argument(
        'robot_id',
        metavar='ROBOT_ID',
        type=str,
        choices=['A', 'B', 'C', 'D'],
        help='Robot ID letter to process (A, B, C, or D).'
    )
    parser.add_argument(
        'position',
        metavar='POSITION',
        type=float,
        help='Target position for the robot (float).'
    )
    parser.add_argument(
        'velocity',
        metavar='VELOCITY',
        type=float,
        help='Movement velocity for the robot (float).'
    )
    args = parser.parse_args()

    run(args.robot_id, args.position, args.velocity)


if __name__ == '__main__':
    main()
