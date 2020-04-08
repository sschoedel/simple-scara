import numpy as np
import math
from pyglet.gl import *

def circle(x, y, radius):
    """
    We want a np.pixel perfect circle. To get one,
    we have to approximate it densely with triangles.
    Each triangle thinner than a np.pixel is enough
    to do it. Sin and cosine are calculated once
    and then used repeatedly to rotate the vector.
    I dropped 10 iterations intentionally for fun.
    """
    iterations = int(2*radius*np.pi)
    s = math.sin(2*np.pi / iterations)
    c = math.cos(2*np.pi / iterations)

    dx, dy = radius, 0

    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(x, y)
    for i in range(iterations+1 - 10):
        glVertex2f(x+dx, y+dy)
        dx, dy = (dx*c - dy*s), (dy*c + dx*s)
    glEnd()

if __name__ == "__main__":
    pass
