# setState 52b 713 486

import collections
import random

# possible moves
_validMoves = ["up", "down", "left", "right"]


def full_move_list(puzzle):
    move_list = []
    curr_node = puzzle

    while curr_node.parentMove is not None:
        move_list.append(curr_node.parentMove)
        curr_node = curr_node.parent

    move_list.reverse()
    return ", ".join(move_list)


# simple function that prints out solve success messages in a consistent format
def print_solve_success(move_count, move_list):
    message = "Successfully found a solution in {} moves. \
              Move list to get to the goal: {}".format(move_count, move_list)
    print(message)


def print_solve_failure(max_nodes):
    print("No solution was found in {} moves.".format(max_nodes))


# Goal state The goal state is "b12 345 678”.
class Puzzle:
    def __init__(self):
        self.separator = " "
#        self.goalState = "b12 345 678"
        self.goalState = "123 456 78b"

        self.currState = self.goalState
        self.maxNodes = 50

        # parent node information
        self.parent = None
        self.parentMove = None

    def set_state(self, state):
        self.currState = state

    # specifies max num of nodes to consider during a search
    # if this limit is exceeded, an error will be thrown
    def set_max_nodes(self, n):
        self.maxNodes = n

    # make n random moves from goal state
    def randomize_state(self, n):
        self.currState = self.goalState
        for i in range(n):
            direction = random.choice(_validMoves)
            if self._is_movement_valid(direction):
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

        if not self._is_movement_valid(direction, blank_index=blank_index):
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
    def _is_movement_valid(self, direction, blank_index=None):
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
            if tile != goal and tile != "b":
                diff += 1
        return diff

    def h2(self):
        accum = 0

        for idx, tile in enumerate(self.currState):
            if tile != self.separator and tile != "b":
                accum += Puzzle.dist(idx, self.goalState.find(tile))

        return accum

    # Evaluation function chosen for beam search
    def beam_eval(self):
        return self.h2()
        # return self.h1() + self.h2()

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

    def solve(self, method, param):
        if method == "A-star":
            if param == "h1":
                self._solve_AStar(lambda puzzle: puzzle.h1())
            elif param == "h2":
                self._solve_AStar(lambda puzzle: puzzle.h2())
            else:
                raise ValueError("Invalid heuristic function")
        elif method == "beam":
            self._solve_beam(int(param))
        else:
            raise ValueError("Not a solve method")

    # solves the puzzle w/ A*
    # Heuristic can be either h1 or h2
    # "h1" is num of misplaced tiles
    # "h2" is dist of tiles from their goal pos
    # prints / returns num moves to solu
    # and seq of moves ("up", "up"...)
    def _solve_AStar(self, heuristic_func):
        if self.currState == self.goalState:
            print("0 moves. At goal state.")

        Node = collections.namedtuple("Node", ["puzzle", "depth", "cost"])

        # Set up initial state

        # list of nodes which have already been expanded and don't need checked
        searched_nodes = []

        # list of nodes which need to be expanded
        # starting as this node = self and f(n) = h(n) as g(n) = 0 at the initial node
        unexplored_nodes = [Node(puzzle=self, depth=0, cost=heuristic_func(self))]

        goal_node = None

        # while we still have nodes to explore, keep looking
        # otherwise, we have failed and must quit
        # Stop exploring if we hit the max node limit
        while unexplored_nodes and len(searched_nodes) <= self.maxNodes:
            # explore the best node (lowest f(n))
            # and remove the best node from the unexplored list
            unexplored_nodes = sorted(unexplored_nodes, key=lambda x: x.cost)
            best_node = unexplored_nodes.pop(0)

            # Check if we've exhausted any more-optimal options
            if goal_node is not None and best_node.cost > goal_node.cost:
                print_solve_success(goal_node.depth, full_move_list(goal_node.puzzle))
                return

            # if the current state isn't in the existing searched nodes, add it
            if not any((best_node.puzzle.currState == node.puzzle.currState for node in searched_nodes)):
                searched_nodes.append(best_node)

            if best_node.puzzle.currState == best_node.puzzle.goalState:
                # set goal_node to the more optimal solution
                if goal_node is None:
                    goal_node = best_node
                else:
                    goal_node = best_node if best_node.cost < goal_node.cost else goal_node

            # Add the neighbors of the best node to the unexplored list
            # so we can explore these nodes in the future
            for neighbor in best_node.puzzle.neighbors():
                # ignore neighbors which have been explored
                if neighbor.currState in (node.puzzle.currState for node in searched_nodes):
                    continue

                new_node_depth = best_node.depth + 1

                # add the new node to the unexplored list
                new_node_cost = new_node_depth + heuristic_func(neighbor)
                new_node = Node(puzzle=neighbor, depth=new_node_depth, cost=new_node_cost)

                unexplored_nodes.append(new_node)

        # if we found a solution, but a more optimal path could still exist
        # print out what we found as the solution
        if goal_node is not None:
            print_solve_success(goal_node.depth, full_move_list(goal_node.puzzle))
        else:
            print_solve_failure(self.maxNodes)

    # solves puzzle with local beam search
    # k is num of states
    # eval func is defined by me and is explained in write up
    # eval func = 0 at goal
    # prints / returns num moves to solu
    # and seq of moves ("up", "up"...)
    # Returns (move count, move list)
    def _solve_beam(self, k):
        if self.currState == self.goalState:
            print("0 moves. At goal state.")

        # Set up initial state

        node_count = 1
        depth = 1
        # Set starting states to self to start working off of
        # as this state is our starting position in the search
        states = [self]

        # stores all states so we know if a repeat occurred
        all_states = {self.currState}

        while node_count <= self.maxNodes:
            new_states = []
            # collect all of the successor states with their f(n) value
            for state in states:
                for neighbor in state.neighbors():
                    node_count += 1

                    # Avoid repeat states
                    if neighbor.currState not in all_states\
                            and node_count < self.maxNodes:
                        new_states.append((neighbor.beam_eval(), neighbor))
                        all_states.add(neighbor.currState)

            new_states = sorted(new_states, key=lambda x: x[0])

            # Pull out the states from the (f(n), state) tuple
            states = list(map(lambda x: x[1], new_states))

            # Checking if the front (best) state is at the goal
            if states[0].currState == states[0].goalState:
                print_solve_success(depth, full_move_list(states[0]))
                print(len(all_states))
                return

            # trim states if needed
            if len(states) > k:
                states = states[:k]

            depth += 1

        print_solve_failure(self.maxNodes)

    # Return Puzzle object for all valid neighboring positions
    def neighbors(self):
        neighbors = []
        for direction in _validMoves:
            if self._is_movement_valid(direction):
                # Create the new neighbor object
                neighboring_node = Puzzle()
                neighboring_node.set_state(self.currState)
                neighboring_node.parent = self
                neighboring_node.parentMove = direction

                # move the neighbor to its successor position
                neighboring_node.move(direction)
                neighbors.append(neighboring_node)

        return neighbors
