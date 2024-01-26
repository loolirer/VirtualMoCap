# Importing modules...
import numpy as np

# Returns the vertices of an unit cube with corner in origin
def cube(position=np.zeros(3).reshape(-1,1), size=1/2):
    vertices = np.array([[(V>>2)&1, (V>>1)&1, (V>>0)&1] for V in range(8)]).T*size+position

    return vertices