#boids.py

import numpy as np

class Boid:
    """
    The Boid class describes the properties for each boid pobject that will be modeled
    """

    def __init__(self,
                init_state = [0,0,1,1],
                size = 0.04):
        self.init_state = np.asarray(init_state, dtype=float)
        self.state = self.init_state.copy()
        self.size = size

    def move_boid(self):
        """
        Moves a boid by adding the velocity to the position
        """
        self.state[:2] += self.state[2:]

    def change_velocity(self, newv_array):
        self.state[2:] = newv_array

class BoidBox:
    """
    Some stuff here
    """

