#boids.py

import numpy as np
from scipy.spatial.distance import pdist, squareform

np.random.seed(0)

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
                init_state = [ 0, 0, 1, 1],
                size = 0.04,
                neighbors = 7
                ):
        self.init_state = np.asarray(init_state, dtype=float)
        self.state = self.init_state.copy()
        self.size = size
        self.neighbors = neighbors

    def move_boid(self):
        """
        Moves a boid by adding the velocity to the position
        """
        self.state[:2] += self.state[2:]

    def change_velocity(self, newv_array):
        self.state[2:] = newv_array

    def choose_velocity(self, flock_state, distance):

        nears = nearest(distance, self.neighbors)

        vecs = np.asarray([flock_state[:, 2:][i] for i in nears])
        xvec = reduce(lambda x, y: x + y, vecs[:,0]) / len(vecs[:,0])
        yvec = reduce(lambda x, y: x + y, vecs[:,1]) / len(vecs[:,1])

        self.change_velocity([xvec, yvec])
        self.move_boid()

class Flock:
    """
    The Flock class reprents a number of Boids
    """
    def __init__(self, flock_size = 10):
        self.flock = np.array([Boid(init_state = [
                    np.random.random(),
                    np.random.random(),
                    np.random.random(),
                    np.random.random()]) for i in range(flock_size)])

        self.flock_state = np.array([i.state for i in self.flock])

    def change_flights(self):
        self.flock_distance = squareform(pdist(self.flock_state[:, :2]))

        for id, boid in enumerate(self.flock):
            boid.choose_velocity(self.flock_state, self.flock_distance[id])
            self.flock[id] = boid

        self.flock_state = np.array([i.state for i in self.flock])

class BoidBox:
    """
    Some stuff here
    """
    def __init__(self,
                 size,
                 bounds = [-2, 2, -2, 2],
                 dt = 1. / 30 ):

        self.flock = Flock(size)
        self.bounds = bounds


        def step(self, dt):
            self.time_elapsed += dt
            flock.change_flights()

            # Check for boundary crossing
            crossed_x1 = (self.flock.flock_state[:, 0] < self.bounds[0])
            crossed_x2 = (self.flock.flock_state[:, 0] > self.bounds[1])
            crossed_y1 = (self.flock.flock_state[:, 1] < self.bounds[2])
            crossed_y2 = (self.flock.flock_state[:, 1] > self.bounds[3])

            #Shoot them out the other side
            self.flock.flock_state[crossed_x1, 0] = self.bounds[0]
            self.flock.flock_state[crossed_x2, 0] = self.bounds[1]
            self.flock.flock_state[crossed_y1, 1] = self.bounds[2]
            self.flock.flock_state[crossed_y2, 1] = self.bounds[3]

            self.flock.flock_state[crossed_x1 | crossed_x2, 0] *= -1
            self.flock.flock_state[crossed_y1 | crossed_y2, 1] *= -1


