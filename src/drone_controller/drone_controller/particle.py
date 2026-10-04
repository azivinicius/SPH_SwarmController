import math
import numpy as np


def W(q1, q2):
    h = q1.h
    k = q1.distance_to(q2) / h

    if 0 <= k <= 1:
        Wk = 1 - 3/2*k**2 + 3/4*k**3
    elif 1 <= k <= 2:
        Wk = 1/4*(2 - k)**3
    else:
        Wk = 0.0

    return 10/(7*math.pi*h**2) * Wk


def grad_W(q1, q2):
    h = q1.h

    dx = q1.x - q2.x
    dy = q1.y - q2.y

    if dx == 0 and dy == 0:
        return 0, 0

    k = q1.distance_to(q2) / h
    dkdx = 1/(h*2*math.sqrt(dx**2 + dy**2)) * 2*dx
    dkdy = 1/(h*2*math.sqrt(dx**2 + dy**2)) * 2*dy

    if 0 <= k <= 1:
        dWdk = -3*k + 9/4*k**2
    elif 1 <= k <= 2:
        dWdk = -3/4*(2 - k)**2
    else:
        dWdk = 0.0

    dWdk = (10/(7*math.pi*h**2)) * dWdk

    dWdx = dWdk*dkdx
    dWdy = dWdk*dkdy

    return dWdx, dWdy


def viscosity(qi, qj):
    zetta = [qi.alpha, qi.beta]   # antes fixo: [1.0, 2.0]
    qij = [qj.x - qi.x, qj.y - qi.y]
    vij = [qj.vel[0] - qi.vel[0], qj.vel[1] - qi.vel[1]]
    pij = (qi.rho + qj.rho) / 2
    cij = (qi.c + qj.c) / 2
    eta_2 = 0.01 * qi.h**2

    vij_qij = np.dot(vij, qij)

    uij = (qi.h * vij_qij) / (qi.distance_to(qj)**2 + eta_2)

    if vij_qij > 0:
        return 0.0
    else:
        visc = 1/pij * (-zetta[0]*cij*uij + zetta[1]*uij**2)
        return visc


class Particle:
    def __init__(self, id, x, y, h, m, rho_0=1000.0,
                 gamma=1, alpha=1.0, beta=2.0, zetta_extra=5.0):

        self.id = id

        self.x = x
        self.y = y

        self.h = h
        self.m = m
        self.rho_0 = rho_0
        self.rho = rho_0

        self.ext_force = [0.0, 0.0]

        self.gamma = gamma
        self.alpha = alpha
        self.beta = beta
        self.zetta_extra = zetta_extra

        self.B = (200 * self.rho * 9.8 * (1/98)) / 1
        self.P = self.B * ((self.rho / self.rho_0)**self.gamma - 1)
        self.c = math.sqrt(self.gamma * (self.P + self.B) / self.rho)

        self.vel = [0.0, 0.0]

    def distance_to(self, q):
        return math.sqrt((q.x - self.x)**2 + (q.y - self.y)**2)

    def pos_norm(self):
        return math.sqrt((self.x)**2 + (self.y)**2)

    def vel_norm(self):
        return math.sqrt((self.vel[0])**2 + (self.vel[1])**2)

    def nearest_distance(self, q_list):
        dists = [self.distance_to(q) for q in q_list if q.id != self.id]
        return min(dists) if dists else float("inf")

    def next_rho(self, q_list):
        """
        Densidade SPH (Eq. 13 do artigo de referencia), AGORA incluindo
        a auto-contribuicao da particula (termo W(0,h)). E por isso que
        o loop nao pula mais 'q.id == self.id' -- cada particula soma
        sua propria contribuicao de kernel, alem da dos vizinhos.
        """
        pi = 0.0
        for q in q_list:
            pi += q.m * W(self, q)
        return pi

    def next_pressure(self):
        if self.rho < 1e-6:
            self.P = 0.0
            self.c = 0.0
            return 0.0

        self.B = 20.0 * self.rho
        P = self.B * ((self.rho / self.rho_0)**self.gamma - 1.0)
        self.c = math.sqrt(self.gamma * (P + self.B) / self.rho)

        return P

    def next_vel(self, q_list):
        # Aqui o loop continua pulando self.id: mesmo se incluido, a
        # contribuicao de grad_W(self, self) e sempre (0,0) (dx=dy=0),
        # entao pular so evita trabalho desnecessario -- nao muda o
        # resultado numerico.
        dvdt = [0.0, 0.0]
        for q in q_list:
            if q.id == self.id:
                continue
            dWdx, dWdy = grad_W(self, q)

            if self.rho < 1e-6 or q.rho < 1e-6:
                P_term = 0.0
            else:
                P_term = (self.P/self.rho**2) + (q.P/q.rho**2) + viscosity(self, q)

            dvdt[0] += q.m * P_term * dWdx
            dvdt[1] += q.m * P_term * dWdy

        dvdt[0] = -dvdt[0] - self.zetta_extra*self.vel[0] + self.ext_force[0]
        dvdt[1] = -dvdt[1] - self.zetta_extra*self.vel[1] + self.ext_force[1]

        return dvdt


    def next_state(self, q_list):
        dvdt = [0.0, 0.0]
        e = 0.0
        pi = 0.0
        P = self.P

        for q in q_list:
            # Density -- agora inclui a auto-contribuicao (sem o 'continue')
            pi += q.m * W(self, q)

            if q.id == self.id:
                continue

            # Pressure
            if self.rho < 1e-6:
                P = 0.0
                self.c = 0.0
            else:
                self.B = 20.0 * self.rho
                P = self.B * ((self.rho / self.rho_0)**self.gamma - 1.0)
                self.c = math.sqrt(self.gamma * (P + self.B) / self.rho)

            # Acceleration
            dWdx, dWdy = grad_W(self, q)

            if self.rho < 1e-6 or q.rho < 1e-6:
                P_term = 0.0
            else:
                P_term = (self.P/self.rho**2) + (q.P/q.rho**2) + viscosity(self, q)

            dvdt[0] += q.m * P_term * dWdx
            dvdt[1] += q.m * P_term * dWdy

            dW = grad_W(self, q)
            vij = [q.vel[0] - self.vel[0], q.vel[1] - self.vel[1]]
            e += q.m * P_term * np.dot(vij, dW)

        e = 1/2 * e

        dvdt[0] = -dvdt[0] - self.zetta_extra*self.vel[0] + self.ext_force[0]
        dvdt[1] = -dvdt[1] - self.zetta_extra*self.vel[1] + self.ext_force[1]

        return pi, P, dvdt, e

    def update_particle(self, rho, P, dvdt,  dt):
        self.rho = rho
        self.P = P
        self.vel[0] += dvdt[0] * dt
        self.vel[1] += dvdt[1] * dt
        self.x += self.vel[0] * dt
        self.y += self.vel[1] * dt