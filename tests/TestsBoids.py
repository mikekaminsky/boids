#TestsBoids.py

import unittest
execfile("boids.py")

class TestBoid(unittest.TestCase):
    
    def setUp(self):
        self.boid = Boid()
    
    def test_size(self):
        self.assertEqual(self.boid.size, 0.04)

    def test_position(self):
        self.assertTrue((self.boid.init_state[0:2] == 0).all())

    def test_move_boid(self):
        self.boid.move_boid()
        self.assertEqual(self.boid.state[0:1], 1)

    def test_change_velocity(self):
        self.boid.change_velocity([2, 2])
        self.assertEqual(self.boid.state[2:3], 2)

suite = unittest.TestLoader().loadTestsFromTestCase(TestBoid)
unittest.TextTestRunner(verbosity=2).run(suite)
