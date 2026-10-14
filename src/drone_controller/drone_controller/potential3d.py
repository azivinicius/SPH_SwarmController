"""3D target potential: the shape the swarm is pulled onto.

Same idea as potential.py, one dimension up.  The controller asks the potential, for every drone,
"which way and how hard should this drone be pushed to reach the target shape?" and adds the answer
(m/s^2) to the SPH acceleration:

    f = potential.force(x, y, z, beta, k)   ->  [fx, fy, fz]

There is ONE class, Potential3D, with two shapes (choose with shape=... or POTENTIAL=sphere|quartic):

    shape='sphere'   sphere of radius R.                                  (the original potential)
    shape='quartic'  closed loop: rounded-star cylinder cut by a tilted plane.   (from potential3d03.py)

The target is where an implicit function alpha is zero; the hill V = alpha^2/2 is zero there and grows away
from it; the force points downhill.  One alpha in 3D gives a SURFACE (the sphere), two alphas give a CURVE
(their intersection: the loop).
"""
import math

import numpy as np


class Potential3D:
    """Target shape centred on (xc, yc, zc).  R is the size of the shape (meaning depends on `shape`).

    shape='sphere'
        R is the radius (m).  The 3D twin of the 2D circle: the force pulls every drone toward the sphere
        surface with the SAME strength k, from inside and from outside; the SPH pressure then spreads the
        drones over the surface.  The force flips sign at the surface, so drones jitter across it by a few
        centimetres (period about 1 s with a 0.1 s command delay).
            s(p)     = |p - c|^2 / R^2 - 1        0 on the sphere, <0 inside, >0 outside
            phi      = s^2                        the hill: 0 on the sphere
            grad phi = 2 s * grad(s)              uphill (away from the sphere)
            force    = -k * grad phi / |grad phi|^beta
        The sphere fixes only the distance to the centre, not the altitude: the drones spread over the
        whole height range zc - R .. zc + R.  Keep zc - R above the floor limit z_min.

    shape='quartic'
        A closed 3D loop: the rounded four-pointed star of potential.Potential (shape='quartic'), cut by a
        tilted plane.  It is where two surfaces meet:
            alpha1(x, y)    = a dx^4 - b dx^2 dy^2 + c dy^4 - R = 0   vertical "star" cylinder (ignores z)
            alpha2(x, y, z) = x/2 + y/2 - z                     = 0   the plane z = (x + y) / 2
        with dx = x - xc, dy = y - yc.  Seen from above it is the 2D star (a = c = 4, b = 6, R = 4:
        corners 1.68 m from the centre on the diagonals, sides 1.00 m on the axes; R is a size constant,
        not a radius: distances scale with R^(1/4), perimeter about 6.95 * R^(1/4) m).  Seen from the
        side its height follows the plane: the (+,+) corner is the highest point and the (-,-) corner the
        lowest, so the loop spans 2.38 m in height (for R = 4), centred at z = (xc + yc) / 2.
            force  = -G * grad(V),  V = (alpha1^2 + alpha2^2) / 2
            G      = G_base + G_a1 |alpha1| + G_a2 |alpha2|     (1, 0, 5): the pull toward the plane gets
                     stronger the farther the drone is from it, the horizontal shape is followed gently
        The force is clipped to length max_force (10), THEN its z part is multiplied by z_gain (100).
        Close to the loop that makes the vertical pull very strong (the drone hits the climb limit max_vz
        whenever it is more than a few centimetres off the plane); far from it the strong horizontal part
        uses up the clip and the z part stays small.  beta, k and zc are ignored: this shape has its own
        gains and its plane does not move with zc.
        WATCH OUT: the plane is not tied to (xc, yc).  It is always z = (x + y) / 2 through the world
        origin.  xc = yc = 0 puts half of the loop below the floor (z from -1.19 to +1.19 m); xc = yc = 1.2
        gives 0.01 .. 2.39 m (touches the floor); xc + yc = 2.4 with both about 1.2 is the loop centred
        1.2 m high.  The plane slope is 0.707 (35 degrees, not 45): z rises 0.5 m per metre in x and in y.
        Too large an R makes the loop chatter around the curve (the real 20 Hz loop with its 0.1 s delay
        cannot follow such a stiff force): R of 1 to 2 converges cleanly, R = 4 does not.
    """

    def __init__(self, xc=0.0, yc=0.0, zc=0.0, R=1.0, shape='sphere', a=4.0, b=6.0, c=4.0, max_force=3.0, gain=1.0,
                 G_base=1.0, G_a1=0.0, G_a2=5.0, z_gain=100.0):
        if shape not in ('sphere', 'quartic'):
            raise ValueError("shape must be 'sphere' or 'quartic'")
        self.xc, self.yc, self.zc, self.R = xc, yc, zc, R      # centre (m) and size (m, see above)
        self.shape = shape
        self.a, self.b, self.c = a, b, c   # quartic only: coefficients (closed shape needs b^2 < 4 a c)
        self.gain = gain                   # quartic only: strength factor applied BEFORE the clip, so max_force stays the cap (1 = original)
        self.max_force = max_force         # quartic only: length clip before the z gain (m/s^2), 10 originally
        self.G_base = G_base               # quartic only: gain floor
        self.G_a1 = G_a1                   # quartic only: extra gain per unit |alpha1| (0 = off)
        self.G_a2 = G_a2                   # quartic only: extra gain per unit |alpha2| (distance to the plane)
        self.z_gain = z_gain               # quartic only: extra factor on the vertical force (100 originally)

    # ------------------------------------------------------------------ sphere
    def s(self, x, y, z):
        """Sphere equation: 0 on the sphere, negative inside, positive outside."""
        return ((x - self.xc) ** 2 + (y - self.yc) ** 2 + (z - self.zc) ** 2) / self.R ** 2 - 1.0

    def phi(self, x, y, z):
        """The sphere's hill: s squared, 0 on the sphere and positive everywhere else."""
        s_val = self.s(x, y, z)
        return s_val * s_val

    def grad_phi(self, x, y, z):
        """Gradient of phi (the uphill direction): 2 s * grad(s)."""
        s_val = self.s(x, y, z)
        grad_s = np.array([2.0 * (x - self.xc) / self.R ** 2,
                           2.0 * (y - self.yc) / self.R ** 2,
                           2.0 * (z - self.zc) / self.R ** 2])
        # grad phi = 2 s grad(s)
        return 2.0 * s_val * grad_s

    # ------------------------------------------------------------------ quartic loop
    def alpha1(self, x, y, z=0):
        """Star-cylinder equation: 0 on the loop's shadow on the floor, negative inside, positive outside."""
        dx = x - self.xc
        dy = y - self.yc
        return self.a*dx**4 - self.b*dx**2*dy**2 + self.c*dy**4 - self.R

    def grad_alpha1(self, x, y, z=0):
        """Derivatives of alpha1 [d/dx, d/dy, d/dz]; the z part is 0 (vertical walls)."""
        dx = x - self.xc
        dy = y - self.yc
        dadx = 4*self.a*dx**3 - 2*self.b*dx*dy**2
        dady = -2*self.b*dx**2*dy + 4*self.c*dy**3
        return np.array([dadx, dady, 0.0])

    def alpha2(self, x, y, z):
        """Plane equation z = (x + y)/2: positive when the drone is BELOW the plane, negative above."""
        return x/2 + y/2 - z

    def grad_alpha2(self, x, y, z=0):
        """Derivatives of alpha2: constant [1/2, 1/2, -1] (the plane's normal)."""
        return np.array([0.5, 0.5, -1.0])

    def V(self, x, y, z=0):
        """The loop's hill (alpha1^2 + alpha2^2) / 2: zero only on the loop."""
        a1 = self.alpha1(x, y)
        a2 = self.alpha2(x, y, z)
        return 0.5*(a1*a1 + a2*a2)

    def grad_V(self, x, y, z=0.0):
        """Uphill direction: alpha1 * grad(alpha1) + alpha2 * grad(alpha2)  (3-vector)."""
        a1 = self.alpha1(x, y)
        a2 = self.alpha2(x, y, z)
        grad_a1 = self.grad_alpha1(x, y)
        grad_a2 = self.grad_alpha2(x, y, z)
        dVdx = a1*grad_a1[0] + a2*grad_a2[0]
        dVdy = a1*grad_a1[1] + a2*grad_a2[1]
        dVdz = a1*grad_a1[2] + a2*grad_a2[2]
        return np.array([dVdx, dVdy, dVdz])

    def _force_quartic(self, x, y, z):
        """Loop force [fx, fy, fz]: -G * grad(V), clipped to max_force in length, then fz * z_gain."""
        a1 = self.alpha1(x, y, z)
        a2 = self.alpha2(x, y, z)

        # adaptive gain: G_base + G_a1*|alpha1| + G_a2*|alpha2|   (G_a1 = 0 in the original file)
        G = self.gain * (self.G_base + self.G_a2 * abs(a2) + self.G_a1 * abs(a1))   # self.gain: f_k / 2, before the clip
        dV = self.grad_V(x, y, z)
        F_conv = -G * dV

        F = F_conv

        if np.linalg.norm(F) > self.max_force:        # keep the direction, cap the length
            F = F / np.linalg.norm(F) * self.max_force

        return [F[0], F[1], self.z_gain*F[2]]

    # ------------------------------------------------------------------ what the controller calls
    def force(self, x, y, z, beta=1.0, k=2.0):
        """Force on a drone at (x, y, z), in m/s^2.

        sphere : -k * grad(phi) / |grad(phi)|^beta   ([0, 0, 0] exactly on the sphere and at its centre,
                 where grad(phi) is 0: avoids 0/0).  beta = 1 keeps only the direction (strength exactly
                 k); beta = 0 keeps the raw gradient.
        quartic: the loop force (beta and k are ignored; the controller sets gain = f_k / 2 and the gain acts
                 before the clip, so POT_K=2 is the original strength and max_force is still the cap).  The order matters: the length clip keeps the
                 direction; only afterwards the vertical part is scaled, so fz is z_gain times a number
                 that is at most max_force (up to 1000 m/s^2 when the clipped force is mostly vertical).
                 The controller limits the vertical COMMAND anyway (max_vz, floor and ceiling guards).
        """
        if self.shape == 'quartic':
            return self._force_quartic(x, y, z)

        g = self.grad_phi(x, y, z)
        norm_g = math.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)
        if norm_g < 1e-9:                         # exactly on the surface (or the centre): no force
            return np.zeros(3)
        f = -k * g / (norm_g ** beta)
        if beta == 0:                             # raw gradient grows with the distance: cap it
            n = math.sqrt(f[0] ** 2 + f[1] ** 2 + f[2] ** 2)
            if n > self.max_force:
                f = f * (self.max_force / n)      # length clip: keeps the direction
        return f