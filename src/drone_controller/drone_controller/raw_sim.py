import argparse

from particle import Particle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class SPHSimulation:
    def __init__(self, n=8, mass=795.2156, h=0.75, rho_0=1000.0, dt=0.05, num_steps=200,
                 xlim=(-15, 15), ylim=(-15, 15), particle_kwargs=None):
        self.n = n
        self.mass = mass
        self.h = h
        self.rho_0 = rho_0
        self.dt = dt
        self.num_steps = num_steps
        self.xlim = xlim
        self.ylim = ylim
        self.particle_kwargs = particle_kwargs or {}

        self.particles = self._create_particles()

        self.time_history = []
        self.avg_rho_history = []
        self.avg_P_history = []
        self.max_vel_history = []
        self.min_vel_history = []
        self.max_accel_history = []
        self.max_gap_history = []
        self.min_gap_history = []
        self.diverged = False

        self.fig = None
        self.ax = None
        self.scat = None
        self.ani = None

    def _create_particles(self):
        pos = np.linspace(-(self.n - 1) / 2.0, (self.n - 1) / 2.0, self.n)
        particles = []
        for i in range(self.n):
            for j in range(self.n):
                q = Particle(
                    str(i) + str(j),
                    pos[i],
                    pos[j],
                    self.h,
                    self.mass,
                    self.rho_0,
                    **self.particle_kwargs
                )
                particles.append(q)
        return particles

    # =====================================================================
    # Passo de simulacao
    # =====================================================================
    def step(self, frame):
        particles = self.particles

        new_rhos = [p.next_rho(particles) for p in particles]
        for p, rho in zip(particles, new_rhos):
            p.rho = rho

        new_Ps = [p.next_pressure() for p in particles]
        for p, P in zip(particles, new_Ps):
            p.P = P

        dvdts = [p.next_vel(particles) for p in particles]
        de_dts = [p.next_energy(particles) for p in particles]

        for p, dvdt, de_dt in zip(particles, dvdts, de_dts):
            p.update_particle(p.rho, p.P, dvdt, de_dt, self.dt)

        if self._check_diverged(dvdts):
            self.diverged = True
            return

        self._record_history(frame, dvdts)

    def _check_diverged(self, dvdts):
        """
        Retorna True se a simulacao explodiu (NaN/Inf em posicao,
        velocidade, densidade, pressao ou aceleracao). Usado pelo
        run_headless() pra abortar cedo e pelo get_cost_metrics() pra
        sinalizar ao CMA-ES que esse conjunto de parametros e ruim.
        """
        for p in self.particles:
            if not np.isfinite(p.rho) or not np.isfinite(p.P):
                return True
            if not np.isfinite(p.x) or not np.isfinite(p.y):
                return True
            if not np.isfinite(p.vel[0]) or not np.isfinite(p.vel[1]):
                return True
        for dvdt in dvdts:
            if not np.isfinite(dvdt[0]) or not np.isfinite(dvdt[1]):
                return True
        return False

    def _record_history(self, frame, dvdts):
        particles = self.particles
        self.time_history.append(frame * self.dt)
        self.avg_rho_history.append(np.mean([p.rho for p in particles]))
        self.avg_P_history.append(np.mean([p.P for p in particles]))

        vels = [np.linalg.norm(p.vel) for p in particles]
        self.max_vel_history.append(np.max(vels))
        self.min_vel_history.append(np.min(vels))

        accels = [np.linalg.norm(dvdt) for dvdt in dvdts]
        self.max_accel_history.append(np.max(accels))

        gaps = [p.nearest_distance(particles) for p in particles]
        self.max_gap_history.append(np.max(gaps))
        self.min_gap_history.append(np.min(gaps))

    # =====================================================================
    # Execucao com animacao (visual, pra rodar e olhar na tela)
    # =====================================================================
    def _setup_figure(self):
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ax.set_xlim(*self.xlim)
        self.ax.set_ylim(*self.ylim)
        self.ax.set_aspect('equal')
        self.ax.set_title('Evolução SPH')
        self.scat = self.ax.scatter([], [], c='blue', s=20)

    def _update_frame(self, frame):
        self.step(frame)
        x = [p.x for p in self.particles]
        y = [p.y for p in self.particles]
        self.scat.set_offsets(np.c_[x, y])
        return self.scat,

    def run(self, show=True, interval=50):
        self._setup_figure()
        self.ani = FuncAnimation(self.fig, self._update_frame,
                                  frames=self.num_steps, interval=interval, blit=True)
        if show:
            plt.show()
        return self.ani

    # =====================================================================
    # Execucao sem GUI (pra usar dentro do cost_fn do CMA-ES, ou pra
    # checar rapido um conjunto de parametros sem abrir janela)
    # =====================================================================
    def run_headless(self):
        for frame in range(self.num_steps):
            self.step(frame)
            if self.diverged:
                break

    def get_cost_metrics(self, rho_0, expected_distance):
        """
        Resume o historico da simulacao nas metricas que a cost_fn do
        CMA-ES precisa. Retorna None se a simulacao divergiu ou nao
        rodou nenhum passo (sinal pra cost_fn devolver penalidade maxima).
        """
        if self.diverged or not self.avg_rho_history:
            return None

        rho_err = np.mean([abs(r - rho_0) for r in self.avg_rho_history])
        p_err = np.mean([abs(p) for p in self.avg_P_history])
        vel_spike = np.max(self.max_vel_history)
        accel_spike = np.max(self.max_accel_history)
        max_gap_ever = np.max(self.max_gap_history)
        min_gap_ever = np.min(self.min_gap_history)

        return {
            "rho_err": rho_err,
            "p_err": p_err,
            "vel_spike": vel_spike,
            "accel_spike": accel_spike,
            "spacing_ceiling": max(0.0, max_gap_ever - expected_distance),
            "spacing_floor": max(0.0, expected_distance - min_gap_ever),
        }

    def print_summary(self, expected_distance=None):
        """
        Imprime um resumo rapido do estado final da simulacao no
        terminal -- util pra checar um conjunto de parametros sem
        precisar abrir os graficos.
        """
        if self.diverged or not self.avg_rho_history:
            print("A simulacao divergiu (NaN/explosao).")
            return

        print("\n=== Resumo da simulacao ===")
        print(f"  densidade media final = {self.avg_rho_history[-1]:.4f} kg/m^3  (alvo: {self.rho_0})")
        print(f"  pressao media final   = {self.avg_P_history[-1]:.4f} Pa")
        print(f"  velocidade maxima     = {max(self.max_vel_history):.4f} m/s")
        print(f"  aceleracao maxima     = {max(self.max_accel_history):.4f} m/s^2")
        print(f"  espacamento minimo    = {min(self.min_gap_history):.4f} m")
        print(f"  espacamento maximo    = {max(self.max_gap_history):.4f} m")
        if expected_distance is not None:
            print(f"  espacamento alvo      = {expected_distance} m")

    # =====================================================================
    # Graficos de historico (densidade, pressao, velocidade)
    # =====================================================================
    def plot_history(self, show=True):
        fig2, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

        axes[0].plot(self.time_history, self.avg_rho_history, 'b-')
        axes[0].set_ylabel('Densidade média')

        axes[1].plot(self.time_history, self.avg_P_history, 'r-')
        axes[1].set_ylabel('Pressão média')

        axes[2].plot(self.time_history, self.max_vel_history, 'g-', label='Máxima')
        axes[2].plot(self.time_history, self.min_vel_history, 'm-', label='Mínima')
        axes[2].set_ylabel('Velocidade')
        axes[2].set_xlabel('Tempo')
        axes[2].legend()

        plt.tight_layout()
        if show:
            plt.show()
        return fig2, axes


def parse_args():
    parser = argparse.ArgumentParser(
        description="Simulacao SPH de enxame de drones -- passe os parametros "
                     "que o CMA-ES encontrou direto pela linha de comando."
    )

    # Parametros de sintonizacao (os que o CMA-ES otimiza)
    parser.add_argument("--gamma", type=float, default=1.0,
                         help="Expoente da equacao de estado de Tait (default: 1.0)")
    parser.add_argument("--alpha", type=float, default=1.0,
                         help="Coeficiente linear da viscosidade artificial (default: 1.0)")
    parser.add_argument("--beta", type=float, default=2.0,
                         help="Coeficiente quadratico da viscosidade artificial (default: 2.0)")
    parser.add_argument("--zetta_extra", type=float, default=5.0,
                         help="Amortecimento extra aplicado a velocidade (default: 5.0)")

    # Parametros fisicos/estruturais da simulacao
    parser.add_argument("--n", type=int, default=8,
                         help="Particulas por lado do grid, total = n^2 (default: 8)")
    parser.add_argument("--mass", type=float, default=795.2156,
                         help="Massa de cada particula em kg (default: 795.2156)")
    parser.add_argument("--h", type=float, default=0.75,
                         help="Comprimento de suavizacao do kernel SPH em m (default: 0.75)")
    parser.add_argument("--rho_0", type=float, default=1000.0,
                         help="Densidade de referencia em kg/m^3 (default: 1000.0)")
    parser.add_argument("--dt", type=float, default=0.05,
                         help="Passo de tempo em segundos (default: 0.05)")
    parser.add_argument("--num_steps", type=int, default=200,
                         help="Numero de passos da simulacao (default: 200)")
    parser.add_argument("--expected_distance", type=float, default=0.5,
                         help="Espacamento-alvo, so para referencia no resumo (default: 0.5)")

    # Modo de execucao
    parser.add_argument("--headless", action="store_true",
                         help="Roda sem abrir a janela de animacao (mais rapido, so imprime resumo e mostra o historico)")
    parser.add_argument("--no-history", action="store_true",
                         help="Nao mostra os graficos de historico ao final")

    return parser.parse_args()


def main():
    args = parse_args()

    sim = SPHSimulation(
        n=args.n, mass=args.mass, h=args.h, rho_0=args.rho_0,
        dt=args.dt, num_steps=args.num_steps,
        particle_kwargs=dict(gamma=args.gamma, alpha=args.alpha,
                              beta=args.beta, zetta_extra=args.zetta_extra),
    )

    print(f"Rodando com gamma={args.gamma}, alpha={args.alpha}, "
          f"beta={args.beta}, zetta_extra={args.zetta_extra}")

    if args.headless:
        print("Modo headless -- sem animacao.")
        sim.run_headless()
    else:
        try:
            sim.run()
        except KeyboardInterrupt:
            print("Simulação interrompida — plotando histórico até aqui.")
        finally:
            plt.close('all')

    sim.print_summary(expected_distance=args.expected_distance)

    if not args.no_history:
        sim.plot_history()


if __name__ == "__main__":
    main()