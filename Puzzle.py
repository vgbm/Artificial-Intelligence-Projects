import random

# possible moves
_validMoves = ["up", "down", "left", "right"]


# Goal state The goal state is "b12 345 678”.
class Puzzle:
    def __init__(self):
        self.separator = " "
        self.goalState = "b12 345 678"
        self.currState = self.goalState
        self.maxNodes = 0

    def set_state(self, state):
        self.currState = state

    # make n random moves from goal state
    def randomize_state(self, n):
        self.currState = self.goalState
        for i in range(n):
            direction = random.choice(_validMoves)
            if self.is_movement_valid(direction):
                self.move(direction)

    def print_state(self):
        print(self.currState)

    # moves the blank tile
    # Dirs include "up", "down", "left", "right"
    def move(self, direction):

        if direction not in _validMoves:
            raise ValueError
            # move otherwise
        self._move_direction(direction)

    def _move_direction(self, direction):
        blank_index = self.currState.find("b")

        if self.is_movement_valid(direction, blank_index = blank_index) == False:
            print("Cannot move {} any further".format(direction))

        swap_tile_index = -1

        if direction == "up":
            swap_tile_index = blank_index - 4
        elif direction == "down":
            swap_tile_index = blank_index + 4
        elif direction == "left":
            swap_tile_index = blank_index - 1
        elif direction == "right":
            swap_tile_index = blank_index + 1

        self._swap(swap_tile_index, blank_index)

    # returns if a given direction is a viable move from the current position
    def is_movement_valid(self, direction, blank_index = None):
        if blank_index is None:
            blank_index = self.currState.find("b")

        if direction == "up" and blank_index <= 3:
            return False
        elif direction == "down" and blank_index >= 7:
            return False
        elif direction == "left" and blank_index in [0, 4, 8]:
            return False
        elif direction == "right" and blank_index in [2, 6, 10]:
            return False

        return True


    def _swap(self, swap_tile_index, blank_index):
        new_state = list(self.currState)
        new_state[blank_index] = new_state[swap_tile_index]
        new_state[swap_tile_index] = "b"

        self.currState = "".join(new_state)

    def h1(self):
        diff = 0
        for tile, goal in zip(self.currState, self.goalState):
            if tile != goal:
                diff += 1
        return diff

    def h2(self):
        accum = 0

        for idx, tile in enumerate(self.currState):
            if tile != self.separator:
                accum += Puzzle.dist(idx, self.goalState.find(tile))

        return accum

    # operates under the idea that |CurrTileIdx - GoalIdx| = 4*a + b amd dist = a + b
    # then we solve a and b for each tile and add it to an accum
    @staticmethod
    def dist(curr_idx, goal_idx):
        row_len = 4

        # get coordinates for the goal pos
        goal_row = int(goal_idx / row_len)
        goal_col = goal_idx % row_len

        # get coordinates for the curr pos
        curr_row = int(curr_idx / row_len)
        curr_col = curr_idx % 4

        # return the absolute dist between the points
        return abs(goal_row - curr_row) + abs(goal_col - curr_col)

    # TODO
    # solves the puzzle w/ A*
    # Heuristic can be either h1 or h2
    # "h1" is num of misplaced tiles
    # "h2" is dist of tiles from their goal pos
    # prints / returns num moves to solu
    # and seq of moves ("up", "up"...)
    def solve_AStar(self, heuristic):
        pass

    # TODO
    # solves puzzle with local beam search
    # k is num of states
    # eval func is defined by me and is explained in write up
    # eval func = 0 at goal
    # prints / returns num moves to solu
    # and seq of moves ("up", "up"...)
    def solve_beam(self, k):
        pass

    # specifies max num of nodes to consider during a search
    # if this limit is exceeded, an error will be thrown
    def set_max_nodes(self, n):
        self.maxNodes = n
