# entry point for project
# handles io for interfacing with the puzzle

import sys
from Puzzle import Puzzle

puzzle = Puzzle()

# list of valid commands
commands = ["setState", "randomizeState", "printState", "move", "solve", "maxNodes"]


# run commands from file
def run_from_file(fileName):
    pass


# run commands from commandline
def run_from_user_input():
    while True:
        command = input("> ")


def exec_command(command):
    commandTokens = command.split(" ")
    if commandTokens[0] not in commands:
        raise ValueError


def print_usage():
    print("This program either takes a file for sending commands \
           or no arguments, indicating we want to perform commands\
           from user input")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        run_from_user_input()
    elif len(sys.argv) == 2:
        run_from_file(sys.argv[1])

    else:
        print_usage()
