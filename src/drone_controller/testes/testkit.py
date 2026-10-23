"""testkit.py -- flight tests as plain scripts, driven by drone_controller.commander.Commander.

    from testkit import Teste

    with Teste.from_args(__doc__) as t:     # Ctrl+C, fence or any error -> Commander.land()
        t.decolar()                         # firmware takeoff service (Commander has no takeoff)
        t.velocidade((0.2, 0.0), 3.0)       # Commander.send_velocity_world((0.2, 0.0), vz) at --rate Hz
        t.parar(2.0)                        # Commander.send_velocity_world((0, 0), vz)
        t.pousar()                          # Commander.land()

Run inside the container (crazyflie_server up, SPH controller NOT running):
    python3 frente_tras.py --v 0.2 --tempo 3
or from the host:  ./dev_tmux.sh test frente_tras --v 0.2      (sim)
                   ./real_tmux.sh test frente_tras --v 0.2     (real)

vz: 0.0 by default (exactly what Commander.send_velocity_world does with no vz).  --kz > 0 adds a
height hold vz = kz * (height - z); Commander then clips the full (vx, vy, vz) to --vmax.
Each run writes src/logs/testes/<timestamp>_<test>.csv (+ .json with the arguments).
"""

import argparse
import csv
import json
import os
import re
import sys
import threading
import time

import rclpy
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_interfaces.srv import Takeoff
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from drone_controller.commander import Commander

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.normpath(os.path.join(HERE, '..', '..', 'logs', 'testes'))


class Abortar(RuntimeError):
    """Safety tripped (fence, lost state, failed takeoff): the test lands."""


class Teste:
    @classmethod
    def from_args(cls, doc=None, add_args=None, argv=None):
        p = argparse.ArgumentParser(description=doc, formatter_class=argparse.RawDescriptionHelpFormatter)
        p.add_argument('--n', type=int, default=0, help='drones cf_0..cf_{n-1} (0 = every /cf_i/state found)')
        p.add_argument('--prefix', default='cf_')
        p.add_argument('--height', type=float, default=0.5, help='takeoff height (m)')
        p.add_argument('--vmax', type=float, default=0.3, help='Commander max_velocity (m/s)')
        p.add_argument('--kz', type=float, default=0.0, help='height hold gain (1/s); 0 = vz 0.0')
        p.add_argument('--rate', type=float, default=20.0, help='send_velocity_world rate (Hz)')
        p.add_argument('--fence', type=float, nargs=4, default=[-1.5, 1.5, -1.5, 1.5],
                       metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'), help='outside -> stop + land')
        p.add_argument('--state-timeout', type=float, default=1.0, help='s without /state -> land')
        p.add_argument('--log-dir', default=LOG_DIR)
        if add_args:
            add_args(p)
        args = p.parse_args(argv)
        return cls(args, os.path.splitext(os.path.basename(sys.argv[0]))[0] or 'teste')

    def __init__(self, args, nome):
        self.args, self.nome = args, nome
        self.names, self.cmd, self.state = [], {}, {}    # cmd[name] = YOUR Commander
        self.voando = False
        self.fase = 'init'
        self._t0 = time.monotonic()
        self._node = self._exec = self._log = None

    # ------------------------------------------------------------------ start / stop
    def __enter__(self):
        try:                               # SIGINT stays a KeyboardInterrupt: ROS survives Ctrl+C, we can land
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init()
        self._node = Node(f'teste_{self.nome}')
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        threading.Thread(target=self._exec.spin, daemon=True).start()   # subscriptions + services
        try:
            self._iniciar()
        except Abortar as e:
            self._fechar()
            print(f'teste nao iniciado: {e}', flush=True)
            raise SystemExit(1)
        except BaseException:
            self._fechar()
            raise
        return self

    def __exit__(self, exc_type, exc, tb):
        acao = ' -> pousando' if self.voando else ''
        if exc_type is KeyboardInterrupt:
            self.info('Ctrl+C' + acao)
        elif exc_type is not None and exc_type is not SystemExit:
            self.info(f'{exc_type.__name__}: {exc}{acao}')
        try:
            if self.voando:
                self.pousar()
        except KeyboardInterrupt:
            self.info('segundo Ctrl+C: saindo SEM pousar')
        finally:
            self._fechar()
        if exc_type is KeyboardInterrupt:
            raise SystemExit(130)
        if exc_type is Abortar:
            raise SystemExit(1)
        return False

    def _iniciar(self):
        a, node = self.args, self._node
        if a.n > 0:
            self.names = [f'{a.prefix}{i}' for i in range(a.n)]
        else:
            pat = re.compile(rf'^/({re.escape(a.prefix)}(\d+))/state$')
            found = {}
            for _ in range(2):                       # second pass: let every topic show up
                t_end = time.monotonic() + 5.0
                while time.monotonic() < t_end and not found:
                    found = {m.group(1): int(m.group(2)) for t, _ty in node.get_topic_names_and_types()
                             if (m := pat.match(t))}
                    time.sleep(0.2)
                time.sleep(1.0)
            if not found:
                raise Abortar('nenhum topico /cf_i/state: o crazyflie_server esta rodando?')
            self.names = sorted(found, key=found.get)

        self._takeoff_cli = {}
        for n in self.names:
            node.create_subscription(LogDataGeneric, f'/{n}/state', lambda m, n=n: self._on_state(n, m), 10)
            self.cmd[n] = Commander(node, n, dt=1.0 / a.rate, max_velocity=a.vmax)
            self._takeoff_cli[n] = node.create_client(Takeoff, f'/{n}/takeoff')

        t_end = time.monotonic() + 5.0
        while time.monotonic() < t_end and len(self.state) < len(self.names):
            time.sleep(0.1)
        if missing := [n for n in self.names if n not in self.state]:
            raise Abortar(f'sem state de {", ".join(missing)}')
        for n in self.names:
            if not self._takeoff_cli[n].wait_for_service(timeout_sec=3.0):
                raise Abortar(f'/{n}/takeoff indisponivel')

        os.makedirs(a.log_dir, exist_ok=True)
        base = os.path.join(a.log_dir, f'{time.strftime("%Y%m%d_%H%M%S")}_{self.nome}')
        with open(base + '.json', 'w') as f:
            json.dump({'teste': self.nome, 'drones': self.names, 'argv': sys.argv[1:], 'args': vars(a)}, f, indent=2)
        self._log_path = base + '.csv'
        self._log_file = open(self._log_path, 'w', newline='')
        self._log = csv.writer(self._log_file)
        self._log.writerow(['t', 'fase', 'drone', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'cmd_vx', 'cmd_vy', 'cmd_vz'])
        self.info(f'drones: {" ".join(self.names)}   log: {self._log_path}')

    def _fechar(self):
        if self._exec is not None:
            self._exec.shutdown(timeout_sec=1.0)
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self._log:
            self._log_file.close()
            self.info(f'log: {self._log_path}')

    def _on_state(self, name, msg):
        if len(msg.values) >= 6:
            self.state[name] = (time.monotonic(), *(float(v) for v in msg.values[:6]))

    # ------------------------------------------------------------------ helpers
    @property
    def node(self):
        """The rclpy node of the test (extra publishers / clients)."""
        return self._node

    def info(self, text):
        print(f'[{time.monotonic() - self._t0:7.2f}] {text}', flush=True)

    def pos(self, name):
        return self.state[name][1:4]

    def _registrar(self, cmds):
        t = f'{time.monotonic() - self._t0:.3f}'
        for n in self.names:
            st = self.state.get(n)
            c = cmds.get(n)
            self._log.writerow([t, self.fase, n, *(f'{v:.4f}' for v in st[1:]),
                                *((f'{v:.4f}' for v in c) if c else ('', '', ''))])

    def _seguranca(self):
        a, now = self.args, time.monotonic()
        xmin, xmax, ymin, ymax = a.fence
        for n in self.names:
            t, x, y = self.state[n][:3]
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                return f'{n} saiu da cerca em x={x:.2f} y={y:.2f}'
            if a.state_timeout > 0 and now - t > a.state_timeout:
                return f'sem state de {n} ha {now - t:.1f} s'
        return None

    def esperar(self, segundos):
        """Sleep without commanding (high-level commander holds), checking the fence."""
        t_end = time.monotonic() + segundos
        while time.monotonic() < t_end:
            if self.voando and (why := self._seguranca()):
                raise Abortar(why)
            self._registrar({})
            time.sleep(1.0 / self.args.rate)

    # ------------------------------------------------------------------ the API used by the tests
    def decolar(self, duracao=3.0):
        h = self.args.height
        self.fase = 'decolar'
        self.info(f'decolando para {h:.2f} m')
        req = Takeoff.Request()
        req.group_mask, req.height = 0, float(h)
        req.duration.sec, req.duration.nanosec = int(duracao), int((duracao % 1) * 1e9)
        self.voando = True
        for n in self.names:
            self._takeoff_cli[n].call_async(req)
        self.esperar(duracao + 1.0)
        if low := [n for n in self.names if self.pos(n)[2] < 0.5 * h]:
            raise Abortar(f'decolagem falhou: {", ".join(low)} abaixo de {0.5 * h:.2f} m')

    def velocidade(self, v, segundos, fase=None):
        """YOUR Commander.send_velocity_world(v, vz) for every drone, at --rate Hz, for `segundos`.
        v = (vx, vy) for all drones, or {name: (vx, vy)}."""
        a = self.args
        if fase is None:
            fase = 'velocidade' if isinstance(v, dict) else f'v=({v[0]:+.2f},{v[1]:+.2f})'
        self.fase = fase
        self.info(f'{self.fase} por {segundos:.1f} s')
        period = 1.0 / a.rate
        t_next = time.monotonic()
        t_end = t_next + segundos
        while time.monotonic() < t_end:
            if why := self._seguranca():
                for n in self.names:
                    self.cmd[n].send_velocity_world((0.0, 0.0), 0.0)
                raise Abortar(why)
            cmds = {}
            for n in self.names:
                vxy = v[n] if isinstance(v, dict) else v
                vz = a.kz * (a.height - self.pos(n)[2]) if a.kz > 0 else 0.0
                self.cmd[n].send_velocity_world(vxy, vz)          # <- your velocity commander
                cmds[n] = (float(vxy[0]), float(vxy[1]), vz)
            self._registrar(cmds)
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()))

    def parar(self, segundos):
        """send_velocity_world((0, 0)) for `segundos`: the drone holds with its own loop."""
        self.velocidade((0.0, 0.0), segundos, fase='parado')

    def pousar(self, duracao=3.0):
        """YOUR Commander.land(): stop streaming, notify_setpoints_stop, then land."""
        self.fase = 'pousar'
        self.info('pousando (Commander.land)')
        time.sleep(2.0 / self.args.rate)                      # no velocity setpoint after this point
        for n in self.names:
            if not self.cmd[n].land(duration_s=duracao):
                self.info(f'{n}: servicos de land indisponiveis')
        self.voando = False
        self.esperar(duracao + 1.5)