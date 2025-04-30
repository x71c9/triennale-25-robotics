#!/usr/bin/env python3

import argparse
import triennale


def run(robot_id):
    print(f"Running for robot_id: {robot_id}")
    robot = triennale.TriennaleRobot(robot_id, mode="read_only")
    length = robot.get_position()

    print(f"Robot cable length in m:")
    print(length)


def main():
    parser = argparse.ArgumentParser(
        description='Process a single robot_id: A, B, C, or D and output its cable length.'
    )
    parser.add_argument(
        'robot_id',
        metavar='ROBOT_ID',
        type=str,
        choices=['A', 'B', 'C', 'D'],
        help='Robot ID letter to process (A, B, C, or D).'
    )
    args = parser.parse_args()

    run(args.robot_id)


if __name__ == '__main__':
    main()
