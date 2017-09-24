# entry point for project
# handles io for interfacing with the puzzle

import sys
from Puzzle import Puzzle


# run commands from file
def run_from_file(filename):
    command_list = []
    puzzle = Puzzle()

    with open(filename, "r") as file:
        for line in file:
            command_list.append(line.strip())

    for command in command_list:
        exec_command(command, puzzle)


# run commands from commandline
def run_from_user_input():
    puzzle = Puzzle()

    while True:
        command = input("> ")
        exec_command(command, puzzle)


# list of valid commands and how many arguments each takes
# defined in the requirements doc
commands = {"setState": 3, "randomizeState": 1, "printState": 0, "move": 1, "solve": 2, "maxNodes": 1}

# confirms a command is valid, and if so, executes it
def exec_command(command, puzzle):
    command_tokens = command.split()

    # if the command does not exist
    if command_tokens[0] not in commands:
        raise ValueError("Not a valid command")
    # if the wrong number of args are present

    if len(command_tokens) - 1 != commands[command_tokens[0]]:
        raise ValueError("Command fed the wrong number of arguments")

    interpret_command(command_tokens, puzzle)


def interpret_command(command_tokens, puzzle):
    base_command = command_tokens[0]

    if base_command == "setState":
        puzzle.set_state(" ".join(command_tokens[1:]))
    elif base_command == "randomizeState":
        puzzle.randomize_state(int(command_tokens[1]))
    elif base_command == "printState":
        puzzle.print_state()
    elif base_command == "move":
        puzzle.move(command_tokens[1])
    elif base_command == "solve":
        puzzle.solve(method=command_tokens[1], param=command_tokens[2])
        pass
    elif base_command == "maxNodes":
        puzzle.set_max_nodes(int(command_tokens[1]))


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
