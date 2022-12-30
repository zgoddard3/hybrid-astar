import numpy as np

from priority_queue import PriorityQueue
from collections import defaultdict
from dubins import CurveList

def rot(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

class HybridAStarSolver:
    def __init__(self, curves : CurveList, heuristics, cell_size, goal_check=None):
        """
        Inputs:
            curves: List of dubins curves to use for the search.
            heuristics: List of heuristics. The solver assumes these are admissible
                and will maximize over them. 
            cell_size: The cell size used to grid the search space (x, y, z, psi). 
                The psi component should perfectly divide 2*pi.
            goal_check: Function to check if we are at the goal which takes in a two 
                arguments. The first being the current cell and the second being the 
                goal cell. By default it only checks position and ignores heading.
        """
        self.curves = curves.copy()
        self.heuristics = heuristics.copy()
        self.cell_size = np.array(cell_size)
        self.goal_check = goal_check or (lambda x, g : all([i == j for i,j in zip(x[:3], g[:3])]))
    
    def solve(self, start, goal):
        """
        Solves for a part from start to goal using Hybrid A*.

        Input:
            start: Initial position and heading (x, y, z, psi)
            goal: Desired final position and heading (x, y, z, psi). By default the 
                heading is ignored. This can be changed via the goal check function 
                in the constructor.
        """
        queue = PriorityQueue()
        costs = defaultdict(lambda : np.inf)
        predecessors = defaultdict(lambda : None)
        closed = set()

        # Convert start to a transformation matrix
        transform = np.eye(4)
        transform[:3,3] = start[:3]
        transform[:2,:2] = rot(start[3])

        # Convert goal to transformation matrix and cell
        goal_transform = np.eye(4)
        goal_transform[:3,3] = goal[:3]
        goal_transform[:2,:2] = rot(goal[3])
        goal_cell = self.get_cell(goal_transform)

        while queue:
            transform = queue.pop()
            cell = self.get_cell(transform)

            if cell in closed:
                continue
            else:
                closed.add(cell)

            if self.goal_check(cell, goal_cell):
                break

            for child, cost, curve in self.children(transform):
                child_cell = self.get_cell(child)
                if child_cell in closed:
                    continue
                
                # Check if new cost is better than old cost.
                # We'll just re-add the node to the queue if it already exists, since heapq
                # doesn't have a convenient way to update keys. To account for this we check
                # if the node is closed right after popping it from the queue
                cost += costs[cell]
                if costs[child_cell] < cost:
                    predecessors[child_cell] = curve, transform
                    costs[child_cell] = cost
                
    
    def children(self, transform):
        out = []
        distance = np.sqrt(3) * self.cell_size
        for curve in self.curves:
            trans = curve.get_transform(distance)
            trans = transform.dot(trans)
            out.append((trans, distance, curve))
        return out
    
    def heuristic(self, start, goal):
        values = [h(start, goal) for h in self.heuristics]
        return max(values)
    
    def get_cell(self, transform):
        """
        Calculates which cell a transformation matrix occupies.
        """
        offset = self.cell_size / 2

        xyz = transform[:3,3] + offset[:3]
        xyz /= self.cell_size[:3]
        cell_xyz = np.floor(xyz).astype(int)

        psi = np.arctan2(transform[1,0], transform[0,0])
        psi += offset[3]
        psi %= 2*np.pi
        psi /= self.cell_size[3]
        cell_psi = np.floor(psi).astype(int)

        return tuple(*cell_xyz, cell_psi)

def euclidean_distance(start, goal):
    pass