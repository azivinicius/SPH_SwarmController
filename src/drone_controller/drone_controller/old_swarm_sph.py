"""All N SPH controllers in ONE process.

Same physics, commands and logs as N `plain_sph` nodes, but one subscription per
drone (N instead of N^2) and density / pressure computed once per tick for the whole
swarm (N^2 kernel calls instead of N^3).  Each tick works in two phases (see
sph_core.SphCore.step): every drone's acceleration is computed from the SAME snapshot,
then every drone is integrated and commanded.

CHANGE vs the previous version: the external potential force is no longer commented
out -- it is controlled by the f_k parameter (POT_K in swarm.conf; 0 = off), exactly
like plain_sph.
"""

import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import LogDataGeneric

from drone_controller.commander import Commander
from drone_controller.sph_ros import build_core, header_latency_ms, read_params


class SwarmSphNode(Node):
    def __init__(self):
        super().__init__('swarm_sph')
        # defaults that differ from plain_sph (the launch file passes everything anyway)
        cfg = read_params(self, {'particle_mass': 3000.0, 'gamma': 1.0, 'zetta_extra': 50.0})
        self._core, names = build_core(
            self, cfg, egos_from_names=lambda n: list(n), density_on_rx=False,
            tag='swarm')
        self._t0 = self.get_clock().now()
        period = cfg['control_period']

        for name in names:
            self.create_subscription(
                LogDataGeneric, f'/{name}/state',
                lambda msg, n=name: self._state_callback(n, msg), 10)
        self._commanders = {
            n: Commander(self, n, dt=period, max_velocity=0.5, max_acceleration=1.0)
            for n in names
        }
        self._timer = self.create_timer(period, self._control_step)
        self.get_logger().info(f'SPH swarm node started: {len(names)} drones in one process.')

    def _now(self):
        return (self.get_clock().now() - self._t0).nanoseconds / 1e9

    def _state_callback(self, name, msg):
        self._core.on_state(name, self._now(), msg.values, header_latency_ms(self, msg))

    def _control_step(self):
        for name, cmd in self._core.step(self._now()).items():
            if cmd is not None:
                self._commanders[name].send_velocity_world(cmd, 0.0)

    def destroy_node(self):
        self._core.close(self._now())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmSphNode()
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
