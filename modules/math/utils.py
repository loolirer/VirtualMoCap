# Importing modules...
import numpy as np

# Returns the vertices of an unit cube with corner in origin
def cube(position=np.zeros(3).reshape(-1,1), size=1/2):
    vertices = np.array([[(V>>2)&1, (V>>1)&1, (V>>0)&1] for V in range(8)]).T*size+position

    return vertices

def skew_symmetric_matrix(t):
    t = t.flatten() # Make array unidimensional

    return np.array([[0, -t[2], t[1]],
                     [t[2], 0, -t[0]],
                     [-t[1], t[0], 0]])

def distance_to_line(P, line):
    a, b, c = line
    x, y = P

    distance = np.abs(a*x +b*y + c) / np.sqrt(a**2 + b**2)

    return distance