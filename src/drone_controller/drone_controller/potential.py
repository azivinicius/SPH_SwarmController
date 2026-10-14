"""2D target potential: the shape the swarm is pulled onto.

A potential here is just a force field.  The controller asks it, for every drone, "which way and how hard
should this drone be pushed to reach the target shape?" and adds the answer (m/s^2) to the SPH acceleration:

    f = potential.force(x, y, beta, k)   ->  [fx, fy]

There is ONE class, Potential, with two shapes (choose with shape=... or POTENTIAL=circle|quartic):

    shape='circle'   circle of radius R.            Constant-strength force (the original potential).
    shape='quartic'  rounded four-pointed star.     Strong far away, clipped (from potential02.py).

In both, the target is where an implicit function alpha(x, y) is zero (negative inside, positive outside).
The "hill" V = alpha^2 / 2 is zero on the shape and grows away from it, and the force points downhill,
so every drone slides toward the shape.
"""
import math

import numpy as np


class Potential:
    """Target shape centred on (xc, yc).  R is the size of the shape (its meaning depends on `shape`).

    shape='circle'
        R is the radius (m).  The force pulls every drone toward the circle with the SAME strength k,
        wherever the drone is (5 cm or 5 m away), from inside and from outside.  Since the force flips
        sign when the drone crosses the circle, the drone never rests exactly on it: it jitters across it
        by a few centimetres.
            s(x, y)  = ((x - xc)^2 + (y - yc)^2) / R^2 - 1     0 on the circle, <0 inside, >0 outside
            phi      = s^2                                      the hill: 0 on the circle, up on both sides
            grad phi = 2 s * grad(s)                            points uphill (away from the circle)
            force    = -k * grad phi / |grad phi|^beta          points downhill (toward the circle)
        beta = 1 (the controller's f_beta) divides by |grad phi|, so only the DIRECTION is kept and the
        strength is exactly k.  beta = 0 keeps the raw gradient (weak near the circle, very strong far
        from it).  k is the controller's f_k.

    shape='quartic'
        A rounded four-pointed star: a dx^4 - b dx^2 dy^2 + c dy^4 = R, with dx = x - xc, dy = y - yc.
        With a = c = 4, b = 6 the four corners are on the diagonals, (4R/(a-b+c))^(1/4) from the centre
        (1.78 m for R = 5), and the four sides are slightly concave, (R/a)^(1/4) from the centre on the
        x and y axes (1.06 m for R = 5).  It stays closed as long as b^2 < 4 a c.  R is NOT a radius: both
        distances scale with R^(1/4); the perimeter is about 6.95 * R^(1/4) m (10.4 m for R = 5), so to
        put N drones d metres apart use R = (N * d / 6.95)^4.
            alpha1 = a dx^4 - b dx^2 dy^2 + c dy^4 - R          0 on the star, <0 inside, >0 outside
            force  = -alpha1 * grad(alpha1), each component clipped to +-max_force
        Near the star the force is small and smooth (no jitter, unlike the circle); far away the quartic
        makes it huge, hence the clip.  beta and k are ignored: this shape has its own gains.
        Too large an R makes the loop chatter around the curve (the real 20 Hz loop with its 0.1 s delay
        cannot follow such a stiff force): R of 1 to 2 converges cleanly, R = 5 does not.
    """

    def __init__(self, xc=0.0, yc=0.0, R=1.0, shape='circle', a=4.0, b=6.0, c=4.0, max_force=3.0, gain=1.0):
        if shape not in ('circle', 'quartic'):
            raise ValueError("shape must be 'circle' or 'quartic'")
        self.xc = xc                  # centre of the shape, world frame (m)
        self.yc = yc
        self.R = R                    # circle: radius (m).  quartic: size constant (see above)
        self.shape = shape
        self.a = a                    # quartic only: coefficients of a dx^4 - b dx^2 dy^2 + c dy^4
        self.b = b
        self.c = c
        self.gain = gain              # quartic only: strength factor applied BEFORE the clip, so max_force stays the cap (1 = original)
        self.max_force = max_force    # quartic only: clip of each force component (m/s^2), 7 originally

    # ------------------------------------------------------------------ circle
    def s(self, x, y):
        """Circle equation: 0 on the circle, negative inside, positive outside."""
        return ((x - self.xc)**2 + (y - self.yc)**2) / self.R**2 - 1.0

    def phi(self, x, y):
        """The circle's hill: s squared, so it is 0 on the circle and positive everywhere else."""
        s_val = self.s(x, y)
        return s_val * s_val

    def grad_phi(self, x, y):
        """Gradient of phi (the uphill direction): 2 s * grad(s)."""
        s_val = self.s(x, y)

        grad_s = np.array([2.0 * (x - self.xc) / self.R**2,
                           2.0 * (y - self.yc) / self.R**2])
        # grad phi = 2 s grad(s)
        return 2.0 * s_val * grad_s

    # ------------------------------------------------------------------ quartic star
    def alpha1(self, x, y, z=0):
        """Star equation: 0 on the star, negative inside, positive outside.  (z is ignored.)"""
        dx = x - self.xc
        dy = y - self.yc
        return self.a*dx**4 - self.b*dx**2*dy**2 + self.c*dy**4 - self.R

    def grad_alpha1(self, x, y, z=0):
        """Derivatives of alpha1 with respect to x and y."""
        dx = x - self.xc
        dy = y - self.yc
        dadx = 4*self.a*dx**3 - 2*self.b*dx*dy**2
        dady = -2*self.b*dx**2*dy + 4*self.c*dy**3
        return [dadx, dady]

    def alpha2(self, z, x=0, y=0):
        """Height hook of the original file: alpha2 = z (note: z comes first).  Not used by force()."""
        return z

    def grad_alpha2(self, x, y, z=0):
        """Derivative of alpha2 = z: the unit vector along z.  force() only reads its x and y parts (0)."""
        return [0.0, 0.0, 1.0]

    def V(self, x, y, z=0):
        """The star's hill: (alpha1^2 + alpha2^2) / 2.  Zero only on the star (at z = 0).

        (The original file passed 0 to alpha2 here and so ignored z; it now uses z.  Same result for the
        default z = 0, and consistent with grad_V.)"""
        a1 = self.alpha1(x, y)
        a2 = self.alpha2(z)

        V = 1/2*a1*a1 + 1/2*a2*a2
        return V

    def grad_V(self, x, y, z=0.0):
        """Uphill direction [dV/dx, dV/dy] = alpha1 * grad(alpha1) (the alpha2 part is zero in x and y)."""
        a1 = self.alpha1(x, y)
        a2 = self.alpha2(z)
        grad_a1 = self.grad_alpha1(x, y)
        grad_a2 = self.grad_alpha2(x, y, 0)

        dVdx = a1*grad_a1[0] + a2*grad_a2[0]
        dVdy = a1*grad_a1[1] + a2*grad_a2[1]

        return [dVdx, dVdy]

    def _force_quartic(self, x, y):
        """Star force [fx, fy]: minus the gradient of V, each component clipped to +-max_force.

        The clip is per axis ("box" clip): fx and fy are limited independently, so the strength can reach
        max_force * 1.41 when both saturate, and when only one saturates the DIRECTION changes slightly.
        The commented lines of the original file show the other option, scaling the vector to length
        max_force (keeps the direction); kept below, disabled.
        The force at the centre (xc, yc) is exactly zero and small around it (it grows with the cube of
        the distance), so a drone exactly at the centre barely moves out of it.
        """
        G = self.gain                           # gain (1 in the original file); applied BEFORE the clip

        dV = self.grad_V(x, y)
        fx = -G*dV[0]
        fy = -G*dV[1]
        norm = math.hypot(fx, fy)
        if norm > self.max_force:
            # fx = fx / norm * self.max_force      (length clip, disabled in the original)
            # fy = fy / norm * self.max_force
            if fx > self.max_force:
                fx = self.max_force
            elif fx < -self.max_force:
                fx = -self.max_force

            if fy > self.max_force:
                fy = self.max_force
            elif fy < -self.max_force:
                fy = -self.max_force
        return [fx, fy]

    # ------------------------------------------------------------------ what the controller calls
    def force(self, x, y, beta=1, k=20):
        """Force on a drone at (x, y), in m/s^2.

        circle : -k * grad(phi) / |grad(phi)|^beta   ([0, 0] exactly on the circle and at its centre,
                 where grad(phi) is 0: avoids 0/0)
        quartic: the clipped star force (beta and k are ignored; the controller sets
                 gain = f_k / 2 and the gain acts before the clip: POT_K=2 is the original strength, a smaller
                 POT_K is weaker, and max_force is still the cap)
        """
        if self.shape == 'quartic':
            return self._force_quartic(x, y)

        g = self.grad_phi(x, y)
        norm_g = math.sqrt(g[0]**2 + g[1]**2)
        if norm_g < 1e-9:
            return np.zeros(2)
        # return -k*g          (raw gradient, same as beta = 0 without the division)
        f = -k * g / (norm_g ** beta)
        if beta == 0:                           # raw gradient grows with the distance: cap it
            n = math.hypot(f[0], f[1])
            if n > self.max_force:
                f = f * (self.max_force / n)    # length clip: keeps the direction
        return f