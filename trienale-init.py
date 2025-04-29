#!/usr/bin/env python3

import argparse
import sys


def run(letters):
    """
    Placeholder for the main logic.
    letters: list of str containing any of 'A', 'B', 'C', 'D'
    """
    # TODO: implement the functionality for each letter
    print(f"Running for letters: {letters}")


def main():
    parser = argparse.ArgumentParser(
        description='Process a string of letters A, B, C, D.'
    )
    parser.add_argument(
        'letters',
        metavar='LETTERS',
        type=str,
        nargs='?',
        help='String of letters to process (e.g., A, ABC, ACD). Defaults to ABCD if omitted.'
    )
    args = parser.parse_args()

    # Use default if none provided
    letters_str = args.letters or 'ABCD'
    # Validate input contains only A-D
    if not set(letters_str).issubset({'A', 'B', 'C', 'D'}):
        parser.error("LETTERS must only contain the characters A, B, C, and D")

    # Convert to list and call run
    letters = list(letters_str)
    run(letters)


if __name__ == '__main__':
    main()
