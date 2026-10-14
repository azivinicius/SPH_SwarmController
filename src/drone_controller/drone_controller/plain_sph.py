"""One SPH controller per drone (launch file starts N of these).

The control loop lives in sph_core.SphCore; this node only connects it to ROS.
Density / pressure of a drone are recomputed when ITS state message arrives
(density_on_rx=True), as before.  See sph_core.py for the new switches and
sph_diag.py for the run folder written to logs/runs/<run_id>/.
"""

import rclpy
from std_srvs.srv import Trigger
from rclpy.node import Node
from crazyflie_interfaces.msg import LogDataGeneric

from drone_controller.commander import Commander
from drone_controller.sph_ros import ControlDriver, build_core, header_latency_ms, read_params
from std_msgs.msg import Empty, Float64MultiArray

class SphPlainNode(Node):
    def __init__(self):
        super().__init__('plain_sph')
        self.declare_parameter('robot_id', 0)
        robot_id = self.get_parameter('robot_id').value
        cfg = read_params(self, {'total_robots': 2})
        self._robot_name = f"{cfg['robot_prefix']}{robot_id}"

        self._core, names = build_core(
            self, cfg, egos_from_names=lambda _names: [self._robot_name],
            tag=f'plain_{self._robot_name}', write_meta=(robot_id == 0))
        self._t0 = self.get_clock().now()
        period = cfg['control_period']

        # own drone: its firmware state.  Other drones: what THEIR nodes publish on /<name>/sph
        self.create_subscription(
            LogDataGeneric, f'/{self._robot_name}/state',
            lambda msg: self._state_callback(self._robot_name, msg), 10)
        for name in names:
            if name != self._robot_name:
                self.create_subscription(
                    Float64MultiArray, f'/{name}/sph',
                    lambda msg, n=name: self._core.on_sph(n, self._now(), msg.data), 10)
        self._sph_pub = self.create_publisher(Float64MultiArray, f'/{self._robot_name}/sph', 10)

        self._commander = Commander(self, self._robot_name, dt=period,
                                    max_velocity=cfg['max_velocity'], max_acceleration=1.0)
        self._driver = ControlDriver(self, cfg, self._core, self._now, self._publish)
        # ros2 service call /<node>/hover std_srvs/srv/Trigger : 0 m/s to the drones, latched
        self.create_service(Trigger, '~/hover', self._hover_srv)
                # one message on /sph/land (Ctrl+b l in tmux, or ./dev_tmux.sh land) lands every drone
        self.create_subscription(Empty, '/sph/land', self._land_cb, 10)
        self.get_logger().info(f'[{self._robot_name}] SPH plain node started.')

    def _now(self):
        return (self.get_clock().now() - self._t0).nanoseconds / 1e9

    def _state_callback(self, name, msg):
        self._core.on_state(name, self._now(), msg.values, header_latency_ms(self, msg))

    def _hover_srv(self, _req, resp):
        self._core.request_hover(self._now())
        resp.success, resp.message = True, 'controller now commands 0 m/s (drones hover); land them by hand'
        return resp
    
    def _land_cb(self, _msg):
        if not self._core.request_land(self._now()):
            return                                    # already landing
        if self._commander.land():
            self.get_logger().info(f'[{self._robot_name}] landing')
        else:
            self.get_logger().warn(f'[{self._robot_name}] land services not available: '
                                   'the controller stopped commanding, but nothing lands the drone')
            
    def _publish(self, out):
        cmd = out.get(self._robot_name)
        if cmd is not None:
            self._commander.send_velocity_world(cmd[:2], cmd[2] if len(cmd) > 2 else 0.0)
        st = self._core.sph_state(self._robot_name)   # own state + the rho / P / c of this tick
        if st is not None:
            self._sph_pub.publish(Float64MultiArray(data=st))

    def destroy_node(self):
        self._driver.close()
        self._core.close(self._now())
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