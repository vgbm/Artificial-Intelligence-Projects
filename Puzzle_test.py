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

if __name__ == '__main__':
    unittest.main()