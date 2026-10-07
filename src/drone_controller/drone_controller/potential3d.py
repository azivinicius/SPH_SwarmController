"""3D target potential: a SPHERE of radius R centred on (xc, yc, zc).

    s(p)    = |p - c|^2 / R^2 - 1          (0 on the sphere, < 0 inside, > 0 outside)
    phi(p)  = s^2                          (zero ON the surface)
    force   = -k * grad(phi) / |grad(phi)|^beta

Same shape as potential.Potential (the 2D circle).  With beta = 1 (the default) the force has
constant magnitude k and always points toward the surface, so a drone is pulled onto the sphere from
inside or outside; the SPH pressure then spreads the drones over it.
"""
import math

import numpy as np


class Potential3D:
    def __init__(self, xc=0.0, yc=0.0, zc=0.0, R=1.0):
        self.xc, self.yc, self.zc, self.R = xc, yc, zc, R

    def s(self, x, y, z):
        return ((x - self.xc) ** 2 + (y - self.yc) ** 2 + (z - self.zc) ** 2) / self.R ** 2 - 1.0

    def phi(self, x, y, z):
        s_val = self.s(x, y, z)
        return s_val * s_val

    def grad_phi(self, x, y, z):
        s_val = self.s(x, y, z)
        grad_s = np.array([2.0 * (x - self.xc) / self.R ** 2,
                           2.0 * (y - self.yc) / self.R ** 2,
                           2.0 * (z - self.zc) / self.R ** 2])
        return 2.0 * s_val * grad_s              # grad phi = 2 s grad(s)

    def force(self, x, y, z, beta=1.0, k=2.0):
        g = self.grad_phi(x, y, z)
        norm_g = math.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)
        if norm_g < 1e-9:                         # exactly on the surface (or the centre): no force
            return np.zeros(3)
        return -k * g / (norm_g ** beta)