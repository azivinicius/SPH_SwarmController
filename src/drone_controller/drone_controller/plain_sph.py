import csv
import os

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import LogDataGeneric

from drone_controller.commander import Commander
from drone_controller.particle import Particle
from drone_controller.potential import Potential



class SphPlainNode(Node):
    CONTROL_PERIOD = 0.05

    def __init__(self):
        super().__init__('plain_sph')

        self.declare_parameter('robot_id', 0)
        self.declare_parameter('robot_prefix', 'cf_')
        self.declare_parameter('total_robots', 2)
        self.declare_parameter('fixed_z', 0.5)
        # self.declare_parameter('particle_mass', 10635.0)
        self.declare_parameter('particle_mass', 7000.0)
        # self.declare_parameter('particle_mass', 6500.0)
        # self.declare_parameter('particle_mass', 7500.0)
        self.declare_parameter('smoothing_h', 3.0)
        self.declare_parameter('rho_0', 1000.0)
        self.declare_parameter('gamma', 7.0)
        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('beta', 2.0)
        # self.declare_parameter('zetta_extra', 5.0)
        self.declare_parameter('zetta_extra', 30.0)

        self.declare_parameter('xc', 2.0)
        self.declare_parameter('yc', 2.0)
        self.declare_parameter('R', 0.5)

        self.declare_parameter('f_beta', 1.0)
        self.declare_parameter('f_k', 2.0)


        self._robot_id = self.get_parameter('robot_id').value
        self._robot_prefix = self.get_parameter('robot_prefix').value
        self._total_robots = self.get_parameter('total_robots').value
        self._fixed_z = self.get_parameter('fixed_z').value
        mass = self.get_parameter('particle_mass').value
        h = self.get_parameter('smoothing_h').value
        rho_0 = self.get_parameter('rho_0').value
        gamma = self.get_parameter('gamma').value
        alpha = self.get_parameter('alpha').value
        beta = self.get_parameter('beta').value
        zetta_extra = self.get_parameter('zetta_extra').value

        xc = self.get_parameter('xc').value
        yc = self.get_parameter('yc').value
        R = self.get_parameter('R').value

        self.f_beta = self.get_parameter('f_beta').value
        self.f_k = self.get_parameter('f_k').value

        self._robot_name = f'{self._robot_prefix}{self._robot_id}'
        self._all_names = [f'{self._robot_prefix}{i}' for i in range(self._total_robots)]

        self._particles = {
            name: Particle(
                id=name, x=0.0, y=0.0, h=h, m=mass,
                rho_0=rho_0, gamma=gamma, alpha=alpha, beta=beta, zetta_extra=zetta_extra,
            )
            for name in self._all_names
        }

        self._known = set()

        for name in self._all_names:
            self.create_subscription(
                LogDataGeneric, f'/{name}/state',
                lambda msg, n=name: self._state_callback(n, msg), 10,
            )

        self._commander = Commander(
            self, self._robot_name, dt=self.CONTROL_PERIOD,
            max_velocity=0.5, max_acceleration=1.0,
        )

        self.potential= Potential(
            xc = xc,
            yc = yc,
            R = R,
        )

        log_dir = '/root/dev_ws/src/logs'
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, f'plain_sph_{self._robot_name}.csv')
        self._log_file = open(log_path, 'w', newline='')
        self._log_writer = csv.writer(self._log_file)
        # self._log_writer.writerow(['t', 'rho', 'pressure', 'ext_force_x', 'ext_force_y', 'vel_x', 'vel_y'])
        self._log_writer.writerow(['t', 'rho', 'pressure', 'ext_force_x', 'ext_force_y',
                           'vel_x', 'vel_y', 'x', 'y'])        
        self._t0 = self.get_clock().now()
        self._timer = self.create_timer(self.CONTROL_PERIOD, self._control_step)
        self._last_tick_time = None
        
        self.get_logger().info(f'[{self._robot_name}] SPH plain node started.')

    def _state_callback(self, name, msg):
        if len(msg.values) < 6:
            return
        x, y, z, vx, vy, vz = msg.values[:6]
        p = self._particles[name]
        p.x, p.y = x, y
        p.vel = [vx, vy]
        self._known.add(name)

    def _control_step(self):
        if self._robot_name not in self._known:
            return

        known_particles = [
            self._particles[n] for n in self._all_names if n in self._known
        ]

        for p in known_particles:
            p.rho = p.next_rho(known_particles)
        for p in known_particles:
            p.P = p.next_pressure()

        ego = self._particles[self._robot_name]
        ego.ext_force = self.potential.force(ego.x, ego.y, self.f_beta, self.f_k)

        neighbors = [p for p in known_particles if p.id != self._robot_name]

        measured_position = (ego.x, ego.y)  # real state, captured before update_particle changes it

        dvdt = ego.next_vel(neighbors)
        e = ego.next_energy(neighbors)
        ego.update_particle(ego.rho, ego.P, dvdt, e, self.CONTROL_PERIOD)

        t = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        # self._log_writer.writerow([t, ego.rho, ego.P, ego.ext_force[0], ego.ext_force[1], ego.vel[0], ego.vel[1]])
        self._log_writer.writerow([t, ego.rho, ego.P, ego.ext_force[0], ego.ext_force[1],
                           ego.vel[0], ego.vel[1],
                           measured_position[0], measured_position[1]])
        # self._commander.send_velocity(measured_position, ego.vel, fixed_z=self._fixed_z)
        self._commander.send_velocity_world(ego.vel,0.0)

    def destroy_node(self):
        self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SphPlainNode()
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