import numpy as np

from priority_queue import PriorityQueue
from collections import defaultdict

class HybridAStarSolver:
    def __init__(self, curves, heuristics, cell_size):
        """
        Inputs:
            curves: List of dubins curves to use for the search.
            heuristics: List of heuristics. The solver assumes these are admissible
                and will maximize over them. 
            cell_size: The cell size used to grid the search space (x, y, z, psi). 
                The psi component should perfectly divide 2*pi.
        """
        self.curves = curves.copy()
        self.heuristics = heuristics.copy()
        self.cell_size = np.array(cell_size)
    
    def solve(self, start, goal):
        """
        Solves for a part from start to goal using Hybrid A*.

        Input:
            start: Initial position and heading (x, y, z, psi)
            goal: Desired final position (x, y, z)
        """
        queue = PriorityQueue()
        costs = defaultdict(lambda : np.inf)
        predecessors = defaultdict(lambda : None)

        while queue:
            node = queue.pop()
    
    def heuristic(self, start, goal):
        values = [h(start, goal) for h in self.heuristics]
        return max(values)
    
    def get_cell(self, transform):
        """
        Calculates which cell a transformation matrix occuppies.
        """
        offset = self.cell_size / 2
        xyz = transform[:3,3] + offset[:3]
        xyz /= self.cell_size[:3]
        cell_xyz = np.floor(xyz).astype(int)

        psi = np.arctan2(transform[1,0], transform[0,0])
        psi += offset[3]
        psi = psi % (2*np.pi)

def euclidean_distance(start, goal):
    pass