"""Observe-only SPH instrumentation: sync particles to real drone poses,
compute SPH density/pressure/force as DIAGNOSTICS, and log everything.

Deliberately has no command publisher of any kind -- it cannot send
cmd_full_state or cmd_velocity_world even by accident. This node only
ever reads /cf_x/state and writes CSV rows.
"""

from typing import Dict, List, Optional, Sequence

import rclpy
from crazyflie_interfaces.msg import LogDataGeneric
from rclpy.node import Node

from drone_controller.particle import Particle
from drone_controller.robot_model import Robot
from drone_controller.sph_logger import SphMetricsLogger


class Controller(Node):
    def __init__(self) -> None:
        super().__init__('sph_monitor')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('robot_prefix', 'cf_')
        self.declare_parameter('total_robots', 3)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('sph_h', 2.0)
        self.declare_parameter('sph_rho0', 1000.0)
        self.declare_parameter('sph_mass', 1800.0)

        self._robot_id = self.get_parameter('robot_id').value
        self._robot_prefix = self.get_parameter('robot_prefix').value
        self._total_robots = self.get_parameter('total_robots').value
        self._control_period = self.get_parameter('control_period').value
        sph_h = self.get_parameter('sph_h').value
        sph_rho0 = self.get_parameter('sph_rho0').value
        sph_mass = self.get_parameter('sph_mass').value

        self._robot_name = f'{self._robot_prefix}{self._robot_id}'
        self._all_names = [f'{self._robot_prefix}{i}' for i in range(self._total_robots)]

        self._robots: Dict[str, Robot] = {}
        self._particles: Dict[str, Particle] = {
            name: Particle(id=name, x=0.0, y=0.0, z=0.0, h=sph_h, m=sph_mass, rho_0=sph_rho0)
            for name in self._all_names
        }

        for name in self._all_names:
            self.create_subscription(
                LogDataGeneric,
                f'/{name}/state',
                lambda msg, agent_name=name: self._state_callback(agent_name, msg),
                10,
            )

        self._log = SphMetricsLogger(self._robot_name)
        self._timer = self.create_timer(self._control_period, self._monitor_step)
        self.get_logger().info(f'SPH monitor (read-only) started for {self._robot_name}.')

    def _state_callback(self, agent_name: str, msg: LogDataGeneric) -> None:
        if len(msg.values) < 6:
            return
        robot = self._robots.setdefault(agent_name, Robot(name=agent_name))
        robot.update_state(*msg.values[:6])

    def _monitor_step(self) -> None:
        ego = self._robots.get(self._robot_name)
        if ego is None:
            return  # haven't heard our own state yet

        known_names = [n for n in self._all_names if n in self._robots]

        # 1. SYNC: every known particle's pose/velocity is overwritten from
        #    the real, measured drone state -- never integrated on its own.
        for name in known_names:
            robot = self._robots[name]
            particle = self._particles[name]
            particle.x, particle.y, particle.z = robot.x, robot.y, robot.z
            particle.vel = [robot.vx, robot.vy, robot.vz]

        known_particles = [self._particles[n] for n in known_names]

        # 2. Compute density + pressure for every known particle (including
        #    ego) from the same synced positions everyone already has --
        #    no need for drones to broadcast their own SPH state.
        for name in known_names:
            p = self._particles[name]
            rho = p.next_rho(known_particles)
            p.set_rho(rho)
            pressure = p.next_pressure()
            p.set_pressure(pressure)

        # 3. SPH-computed acceleration for ego, as a DIAGNOSTIC value only.
        ego_particle = self._particles[self._robot_name]
        neighbor_particles = [self._particles[n] for n in known_names if n != self._robot_name]
        neighbor_data = {p.id: (p.rho, p.P, p.c) for p in neighbor_particles}
        sph_accel = ego_particle.next_vel(neighbor_particles, neighbor_data)

        self._log.log(
            robot_name=self._robot_name,
            rho=ego_particle.rho, pressure=ego_particle.P, sound_speed=ego_particle.c,
            x=ego_particle.x, y=ego_particle.y, z=ego_particle.z,
            vx=ego_particle.vel[0], vy=ego_particle.vel[1], vz=ego_particle.vel[2],
            sph_ax=sph_accel[0], sph_ay=sph_accel[1], sph_az=sph_accel[2],
            cmd_ax=0.0, cmd_ay=0.0, cmd_az=0.0,
            n_known_neighbors=len(neighbor_particles), n_total_neighbors=self._total_robots - 1,
            real_x=ego.x, real_y=ego.y, real_z=ego.z,
            real_vx=ego.vx, real_vy=ego.vy, real_vz=ego.vz,
        )

    def destroy_node(self):
        self._log.close()
        super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()