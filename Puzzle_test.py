import unittest
from Puzzle import Puzzle


class TestPuzzle(unittest.TestCase):
    def test_h1(self):
        puzzle = Puzzle()
        puzzle.currState = "724 5b6 831"

        dist = puzzle.h1()
        self.assertEqual(8, dist)

    def test_h2(self):
        puzzle = Puzzle()
        puzzle.currState = "724 5b6 831"

        manhatten_dist = puzzle.h2()
        self.assertEqual(18, manhatten_dist)

    ######################
    # Basic search tests #
    ######################
    def test_basic_a_star_with_h1(self):
        puzzle = Puzzle()
        puzzle.currState = "b42 135 678"

        move_count = puzzle.solve("A-star", "h1")
        self.assertEqual(4, move_count)

    def test_basic_a_star_with_h2(self):
        puzzle = Puzzle()
        puzzle.currState = "b42 135 678"

        move_count = puzzle.solve("A-star", "h2")
        self.assertEqual(4, move_count)

    def test_basic_beam_with_high_k(self):
        puzzle = Puzzle()
        puzzle.currState = "b42 135 678"

        move_count = puzzle.solve("beam", "1000")

        # expect the solution to be optimal since the solution is short and k is high
        self.assertEqual(4, move_count)

    def test_basic_beam_with_low_k(self):
        puzzle = Puzzle()
        puzzle.currState = "b42 135 678"

        move_count = puzzle.solve("beam", "1")

        # expect the solution to be optimal only because the solution is so short
        self.assertEqual(4, move_count)


if __name__ == '__main__':
    unittest.main()
