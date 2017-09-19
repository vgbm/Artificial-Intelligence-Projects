#entry point for project
#handles io for interfacing with the puzzle

import sys
from Puzzle import Puzzle

puzzle = Puzzle()

#list of valid commands
commands = ["setState", "randomizeState", "printState", 
	    "move", "solve", "maxNodes"]

#run commands from file
def runFromFile(fileName):
	pass

#run commands from commandline
def runFromUserInput():
	while True:
		command = input("> ")

def execCommand(command):
		commandTokens = command.split(" ")
		if commandTokens[0] not in commands:
			raise ValueError
		

def printUsage():
	print("This program either takes a file for sending commands \
	or no arguments, indicating we want to perform commands\
	from user input")

if __name__ == "__main__" :
	if len(sys.argv) == 1 :
		runFromUserInput()	
	elif len(sys.argv) == 2 :
		runFromFile(sys.argv[1])

	else:
		printUsage()