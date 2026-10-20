"""3D SPH particle: same interface as particle.Particle, with z.

Differences from the 2D particle (everything else -- pressure law B = 20 rho, artificial viscosity,
damping zetta_extra, sph_gain -- is the same maths):

  * kernel: cubic spline with the 3D normalisation sigma = 1/(pi h^3)   (2D uses 10/(7 pi h^2)).
    Density rho_i = sum_j m_j W_ij therefore has different units than in 2D: the particle mass that gives
    rho_0 at the wanted spacing is NOT the 2D one (gen_config.py --dim 3 solves it).
  * z_stretch: anisotropy.  Distances inside the kernel use dz / z_stretch, so with z_stretch = 2 two
    drones stacked vertically start repelling at twice the horizontal distance.  Use it when drones
    must keep more vertical room (downwash of the upper one hits the lower one).
  * vel, ext_force and every acceleration term are 3-vectors.
"""
import math


def _w_raw(k):
    if k <= 1.0:
        return 1.0 - 1.5 * k * k + 0.75 * k ** 3
    if k <= 2.0:
        return 0.25 * (2.0 - k) ** 3
    return 0.0


def _dw_raw(k):
    if k <= 1.0:
        return -3.0 * k + 2.25 * k * k
    if k <= 2.0:
        return -0.75 * (2.0 - k) ** 2
    return 0.0


def kernel3(r, h):
    """W(r, h) of the 3D cubic spline (r = distance, already in kernel metric)."""
    return _w_raw(r / h) / (math.pi * h ** 3)


class Particle3D:
    dim = 3

    def __init__(self, id, x, y, z, h, m, rho_0=1000.0, gamma=1, alpha=1.0, beta=2.0,
                 zetta_extra=5.0, sph_gain=1.0, z_stretch=1.0, obstacle=False):
        self.id = id
        self.x, self.y, self.z = x, y, z
        self.h, self.m, self.rho_0 = h, m, rho_0
        self.rho = rho_0
        self.ext_force = [0.0, 0.0, 0.0]
        self.gamma, self.alpha, self.beta = gamma, alpha, beta
        self.zetta_extra, self.sph_gain = zetta_extra, sph_gain
        self.z_stretch = max(float(z_stretch), 1e-6)
        self.last_terms = None
        self.B = (20.0/self.gamma) * self.rho
        self.P = self.B * ((self.rho / self.rho_0) ** self.gamma - 1)
        self.c = math.sqrt(self.gamma * (self.P + self.B) / self.rho)
        self.vel = [0.0, 0.0, 0.0]
        self.obstacle = obstacle

    # ---- geometry ------------------------------------------------------------------------------
    def distance_to(self, q):
        """Real 3D centre distance (logs, collisions)."""
        return math.sqrt((q.x - self.x) ** 2 + (q.y - self.y) ** 2 + (q.z - self.z) ** 2)

    def kdist(self, q):
        """Distance in the kernel metric (z divided by z_stretch)."""
        dz = (q.z - self.z) / self.z_stretch
        return math.sqrt((q.x - self.x) ** 2 + (q.y - self.y) ** 2 + dz * dz)

    def nearest_distance(self, q_list):
        d = [self.distance_to(q) for q in q_list if q.id != self.id]
        return min(d) if d else float('inf')

    def pos_norm(self):
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def vel_norm(self):
        return math.sqrt(sum(v * v for v in self.vel))

    # ---- kernel --------------------------------------------------------------------------------
    def W(self, q):
        return kernel3(self.kdist(q), self.h)

    def grad_W(self, q):
        """d W_ij / d (x_i, y_i, z_i)  -- gradient w.r.t. the REAL coordinates of i."""
        s = self.z_stretch
        dx, dy, dzs = self.x - q.x, self.y - q.y, (self.z - q.z) / s
        r = math.sqrt(dx * dx + dy * dy + dzs * dzs)
        if r < 1e-12:
            return 0.0, 0.0, 0.0
        k = r / self.h
        dWdr = _dw_raw(k) / (math.pi * self.h ** 3) / self.h
        f = dWdr / r
        return f * dx, f * dy, f * dzs / s

    def viscosity(self, q):
        qij = (q.x - self.x, q.y - self.y, (q.z - self.z) / self.z_stretch)
        vij = (q.vel[0] - self.vel[0], q.vel[1] - self.vel[1], (q.vel[2] - self.vel[2]) / self.z_stretch)
        vq = vij[0] * qij[0] + vij[1] * qij[1] + vij[2] * qij[2]
        if vq > 0:
            return 0.0
        pij = (self.rho + q.rho) / 2
        cij = (self.c + q.c) / 2
        r2 = qij[0] ** 2 + qij[1] ** 2 + qij[2] ** 2
        uij = (self.h * vq) / (r2 + 0.01 * self.h ** 2)
        return 1 / pij * (-self.alpha * cij * uij + self.beta * uij ** 2)

    # ---- SPH -----------------------------------------------------------------------------------
    def next_rho(self, q_list):
        return sum(q.m * self.W(q) for q in q_list)          # includes the self term, like the 2D code

    def next_pressure(self):
        if self.rho < 1e-6:
            self.P = 0.0
            self.c = 0.0
            return 0.0
        self.B = (20.0/self.gamma) * self.rho
        P = self.B * ((self.rho / self.rho_0) ** self.gamma - 1.0)
        self.c = math.sqrt(self.gamma * (P + self.B) / self.rho)
        return P

    def accel_terms(self, q_list):
        a_press, a_visc = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        for q in q_list:
            if q.id == self.id:
                continue
            if self.rho < 1e-6 or q.rho < 1e-6:
                continue
            g = self.grad_W(q)
            p_term = (self.P / self.rho ** 2) + (q.P / q.rho ** 2)
            v_term = self.viscosity(q)
            for a in range(3):
                a_press[a] -= q.m * p_term * g[a]
                a_visc[a] -= q.m * v_term * g[a]
        a_damp = [-self.zetta_extra * v for v in self.vel]
        a_ext = [float(e) for e in self.ext_force]
        return a_press, a_visc, a_damp, a_ext

    def next_vel(self, q_list):
        a_press, a_visc, a_damp, a_ext = self.accel_terms(q_list)
        g = self.sph_gain
        total = [g * (a_press[i] + a_visc[i]) + a_damp[i] + a_ext[i] for i in range(3)]
        self.last_terms = {'press': a_press, 'visc': a_visc, 'damp': a_damp, 'ext': a_ext, 'total': total}
        return total

    def update_particle(self, rho, P, dvdt, dt):
        if self.obstacle:
            return                    # obstacles don't move
        self.rho, self.P = rho, P
        for i in range(3):
            self.vel[i] += dvdt[i] * dt
        self.x += self.vel[0] * dt
        self.y += self.vel[1] * dt
        self.z += self.vel[2] * dt