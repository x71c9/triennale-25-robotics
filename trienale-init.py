#!/usr/bin/env python3

import argparse


def run(letters):
    """
    Placeholder for the main logic.
    letters: list of str containing any of 'A', 'B', 'C', 'D'
    """
    # TODO: implement the functionality for each letter
    print(f"Running for letters: {letters}")




def main():
    parser = argparse.ArgumentParser(
        description='Process letters A, B, C, D.'
    )
    parser.add_argument(
        'letters',
        metavar='LETTER',
        type=str,
        nargs='*',
        choices=['A', 'B', 'C', 'D'],
        help='Letters to process (A, B, C, D). If none specified, all are used.'
    )
    args = parser.parse_args()

    # If no arguments passed, default to all letters
    letters = args.letters if args.letters else ['A', 'B', 'C', 'D']

    run(letters)


if __name__ == '__main__':
    main()
