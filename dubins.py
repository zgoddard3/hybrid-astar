import numpy as np
from scipy.linalg import expm

class DubinsCurve:
    def __init__(self, radius, angle, speed):
        self._radius = radius
        self._angle = angle
        self._speed = speed

        # Calculate turn rate
        turn_rate = speed / radius

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
        time = self.get_time(distance)
        return expm(self._twist * time)