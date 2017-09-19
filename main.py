#entry point for project
#handles io for interfacing with the puzzle

import sys

#run commands from file
def runFromFile(fileName):
	pass

#run commands from commandline
def runFromUserInput():
	pass

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
