import numpy as np
import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import LogDataGeneric

from drone_controller.commander import Commander


class HelloNode(Node):
    KP = 0.5
    KD = 0.001
    CONTROL_PERIOD = 0.05  # fixed, matches Commander's dt exactly

    def __init__(self):
        super().__init__('hello_node')
        self.declare_parameter('robot_name', 'cf_0')
        self.declare_parameter('target', [0.8, 0.0, 0.5])
        self._robot_name = self.get_parameter('robot_name').value
        self._target = np.array(self.get_parameter('target').value, dtype=float)

        self.get_logger().info(f'[{self._robot_name}] target: {self._target}')

        self._commander = Commander(
            self, self._robot_name, dt=self.CONTROL_PERIOD,
            max_velocity=0.5, max_acceleration=1.0,
        )
        self._latest_state = None  # (position, velocity) -- cached only
        self._sub = self.create_subscription(
            LogDataGeneric, f'/{self._robot_name}/state', self._state_callback, 10
        )
        self._timer = self.create_timer(self.CONTROL_PERIOD, self._control_step)
        self._tick_count = 0

    def _state_callback(self, msg: LogDataGeneric) -> None:
        x, y, z, vx, vy, vz = msg.values[:6]
        self._latest_state = (np.array([x, y, z]), np.array([vx, vy, vz]))

    def _control_step(self) -> None:
        if self._latest_state is None:
            return
        position, velocity = self._latest_state

        v_des_xy = self.KP * (self._target[:2] - position[:2])
        self._commander.send_velocity(position[:2], v_des_xy, fixed_z=self._target[2])

        self._tick_count += 1
        if self._tick_count % 20 == 0:
            self.get_logger().info(
                f'[{self._robot_name}] real: {position[0]:.3f} {position[1]:.3f} {position[2]:.3f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
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