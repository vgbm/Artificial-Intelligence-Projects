class Puzzle():
	#Goal state The goal state is "b12 345 678”.

	validMoves = ["up", "down", "left", "right"]

	def __init__(self):
		self.currState = "bbb bbb bbb"
		self.goalState = "b12 345 678"
		self.maxNodes = 0

	def setState(self, state):
		self.currState = state

	#TODO
	#make n random moves from goal state
	def randomState(self, n):
		pass

	def printState(self):
		print(self.currState)

	#TODO
	#moves the blank tile
	#Dirs include "up", "down", "left", "right"
	def move(self, direction):
		if direction not in validMoves:
			raise ValueError
		#move otherwise

	#TODO
	#solves the puzzle w/ A*
	#Heuristic can be either "h1" or "h2"
	#"h1" is num of misplaced tiles
	#"h2" is dist of tiles from their goal pos
	#prints / returns num moves to solu 
	#and seq of moves ("up", "up"...)
	def solveAStar(self, heuristic):
		pass

	#TODO
	#solves puzzle with local beam search
	#k is num of states
	#eval func is defined by me and is explained in write up
	#eval func = 0 at goal
	#prints / returns num moves to solu 
	#and seq of moves ("up", "up"...)
	def solveBeam(self, k):
		pass

	#specifies max num of nodes to consider during a search
	#if this limit is exceeded, an error will be thrown
	def setMaxNodes(self, n):
		self.maxNodes = n
