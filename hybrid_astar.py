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
        self.distance = np.sqrt(3) * self.cell_size[:3].max()
        self.goal_check = goal_check or (lambda x, g : all([i == j for i,j in zip(x[:3], g[:3])]))

        self.queue = PriorityQueue(self.get_cell)
        self.costs = defaultdict(lambda : np.inf)
        self.predecessors = defaultdict(lambda : None)
        self.closed = set()

        self.final = None
    
    def solve(self, start, goal):
        """
        Solves for a part from start to goal using Hybrid A*.

        Input:
            start: Initial position and heading (x, y, z, psi)
            goal: Desired final position and heading (x, y, z, psi). By default the 
                heading is ignored. This can be changed via the goal check function 
                in the constructor.
        """
        
        self.queue.clear()
        self.costs.clear()
        self.predecessors.clear()
        self.closed.clear()

        # Convert start to a transformation matrix
        transform = np.eye(4)
        transform[:3,3] = start[:3]
        transform[:2,:2] = rot(start[3])

        # Convert goal to transformation matrix and cell
        goal_transform = np.eye(4)
        goal_transform[:3,3] = goal[:3]
        goal_transform[:2,:2] = rot(goal[3])
        goal_cell = self.get_cell(goal_transform)

        self.queue.push(self.heuristic(transform, goal_transform), transform)
        cell = self.get_cell(transform)
        self.costs[cell] = 0

        while self.queue:
            transform = self.queue.pop()
            cell = self.get_cell(transform)
            self.closed.add(cell)

            if self.goal_check(cell, goal_cell):
                self.final = transform
                break

            for child, curve in self.children(transform):
                child_cell = self.get_cell(child)
                if child_cell in self.closed:
                    continue

                cost = self.costs[cell] + self.distance
                if self.costs[child_cell] > cost:
                    self.predecessors[child_cell] = curve, transform
                    self.costs[child_cell] = cost
                    self.queue.push(cost + self.heuristic(child, goal_transform), child)
    
    def children(self, transform):
        out = []
        for curve in self.curves:
            trans = curve.get_transform(self.distance)
            trans = transform.dot(trans)
            out.append((trans, curve))
        return out
    
    def heuristic(self, start, goal):
        values = [h(start, goal) for h in self.heuristics]
        return max(values)
    
    def get_cell(self, transform):
        """
        Calculates which cell a transformation matrix occupies.
        """
        offset = self.cell_size / 2
        cell = [0] * 4

        xyz = transform[:3,3] + offset[:3]
        xyz /= self.cell_size[:3]
        cell[:3] = np.floor(xyz).astype(int)

        psi = np.arctan2(transform[1,0], transform[0,0])
        psi += offset[3]
        psi %= 2*np.pi
        psi /= self.cell_size[3]
        cell[3] = np.floor(psi).astype(int)

        return tuple(cell)
    
    def get_path(self, goal=None):
        if goal is None:
            goal = self.final
        cell = self.get_cell(goal)
        traj = []
        while self.predecessors[cell] is not None:
            curve, transform = self.predecessors[cell]
            traj.append(curve.get_trajectory(self.distance, init_trans=transform)[0])
            cell = self.get_cell(transform)
        
        traj.reverse()
        return np.vstack(traj)


def euclidean_distance(start, goal):
    return np.linalg.norm(goal[:3,3] - start[:3,3])

if __name__ == "__main__":
    from itertools import product
    from dubins import DubinsCurve

    primitives = []
    speed = 1.0
    turn_rates = [-1., 0., 1.]
    angles = [-.5, 0., .5]
    for rate, angle in product(turn_rates, angles):
        primitives.append(DubinsCurve(rate, angle, speed))

    solver = HybridAStarSolver(primitives, [euclidean_distance], (1,1,1,np.pi/4))

    start = (0,0,0,0)
    goal = (5,5,5,0)

    solver.solve(start, goal)

    path = solver.get_path()

    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(*path.T)
    ax.scatter(goal[0], goal[1], goal[2], color='green', s=100)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_aspect('equal')
    plt.savefig("example_path.png")
    plt.show()