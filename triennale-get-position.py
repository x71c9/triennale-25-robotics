#!/usr/bin/env python3

import argparse
import sys

import triennale


def run(robot_id):
    print(f"Running for robot_id: {robot_id}")
    # robot = triennale.TriennaleRobot(robot_id)
    # robot.get_position()


def main():
    parser = argparse.ArgumentParser(
        description='Process a single robot_id: A, B, C, or D.'
    )
    parser.add_argument(
        'robot_id',
        metavar='ROBOT_ID',
        type=str,
        choices=['A', 'B', 'C', 'D'],
        help='Robot ID letter to process (A, B, C, or D).'
    )
    args = parser.parse_args()

    robot_id = args.robot_id

    run(robot_id)


if __name__ == '__main__':
    main()
