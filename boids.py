#boids.py

import numpy as np
from scipy.spatial.distance import pdist, squareform


def nearest(dists,n):
  """Find the nearest n neighbors """
  obj = []
  mins = dists.tolist()[:n]
  mins.sort()
  for i in dists[n:]:
      if i < mins[-1]: 
          mins.append(i)
          mins.sort()
          mins= mins[:n]
  for i in mins:
    loc = dists.tolist().index(i) 
    obj.append(loc)
  return obj



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

    def choose_velocity(self, 
                        choose_type = "nearest",
                        neighbors = 7):



class Flock:
    """
    The Flock class reprents a number of Boids
    """
    def __init__(self,
                 flock_size = 10):
        self.flock = np.array([Boid() for i in range(flock_size)])
        self.flock_state = np.array([i.state for i in flock])

        self.flock_distance = squareform(pdist(self.flock_state[:, :2]))



class BoidBox:
    """
    Some stuff here
    """

