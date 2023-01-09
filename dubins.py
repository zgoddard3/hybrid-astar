import numpy as np
from scipy.linalg import expm

class DubinsCurve:
    def __init__(self, turn_rate, angle, speed):
        self._radius = np.inf if turn_rate == 0 else abs(speed / turn_rate)
        self._angle = angle
        self._speed = speed

        # Set up twist matrix
        self._twist = np.zeros((4,4))
        self._twist[:3,3] = np.cos(angle), 0 , np.sin(angle)
        self._twist[:3,3] *= speed
        self._twist[1,0] = turn_rate
        self._twist[0,1] = -turn_rate
    
    @property
    def radius(self):
        return self._radius
    
    @property
    def angle(self):
        return self._angle
    
    @property
    def speed(self):
        return self._speed
    
    def get_time(self, distance):
        return distance / self._speed
    
    def get_transform(self, distance):
        # This is not particularly efficient computationally,
        # but it's a quick way to do it for now.
        time = self.get_time(distance)
        return expm(self._twist * time)
    
    def get_trajectory(self, distance, step=0.1, init_trans=None):
        if init_trans is None:
            init_trans = np.eye(4)

        distances = np.arange(0, distance, step)
        points = np.zeros((distances.size, 3))
        for i, d in enumerate(distances):
            trans = init_trans.dot(self.get_transform(d))
            points[i] = trans[:3,3]
        return points, init_trans.dot(self.get_transform(distance))

CurveList = list[DubinsCurve]

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from itertools import product
    from random import shuffle

    primitives : CurveList = []
    speed = 1.0
    turn_rates = [-1., 0., 1.]
    angles = [-.5, 0., .5]
    for rate, angle in product(turn_rates, angles):
        primitives.append(DubinsCurve(rate, angle, speed))

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    
    for curve in primitives:
        points, _ = curve.get_trajectory(1.5)
        plt.plot(*points.T)
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    plt.show(block=False)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    
    shuffle(primitives)
    trans = np.eye(4)
    traj = []
    for curve in primitives:
        points, trans = curve.get_trajectory(1.5, init_trans=trans)
        traj.append(points)
    traj = np.vstack(traj)
    plt.plot(*traj.T)
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_aspect('equal')
    plt.show()