"""Builds and publishes Crazyflie FullState messages.

send_accel ports Theo's AccelerationCommander pattern exactly: caller
supplies a_des (computed from real state, closed loop, one layer above
this class); this integrates it into a self-consistent position/
velocity/acceleration triple using a FIXED dt, not measured wall-clock
time -- the caller must drive it from a fixed-rate timer, not directly
from a subscription callback.

send_state is the pure, stateless message builder underneath everything
else -- no memory, no computation, just publishes exactly what it's given.
"""

import math
from typing import Optional, Tuple

import numpy as np
from crazyflie_interfaces.msg import FullState, VelocityWorld
from rclpy.node import Node

from crazyflie_interfaces.srv import Land, NotifySetpointsStop


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class Commander:
    def __init__(self, node, robot_name, dt, max_velocity=None, max_acceleration=None):
        self._node = node
        self._dt = dt  # fallback only, for the very first tick
        self._max_velocity = max_velocity
        self._max_acceleration = max_acceleration
        self._p_ref = None
        self._v_ref = None
        self._last_time = None
        self._publisher = node.create_publisher(FullState, f'/{robot_name}/cmd_full_state', 10)
        self._velocity_world_publisher = node.create_publisher(
            VelocityWorld, f'/{robot_name}/cmd_velocity_world', 10
        )
        # landing through the firmware's high-level commander (services of crazyflie_server)
        self._notify_cli = node.create_client(NotifySetpointsStop, f'/{robot_name}/notify_setpoints_stop')
        self._land_cli = node.create_client(Land, f'/{robot_name}/land')

    def land(self, duration_s: float = 3.0, height: float = 0.0) -> bool:
        """Hand the drone back to the high-level commander, then land it.  The caller must have
        STOPPED publishing velocity setpoints.  notify_setpoints_stop lowers the priority of the
        streamed setpoints; land is sent only after it returned (otherwise it could be ignored).
        False if crazyflie_server's services are not available (e.g. particle_sim)."""
        if not (self._notify_cli.service_is_ready() and self._land_cli.service_is_ready()):
            return False
        stop = NotifySetpointsStop.Request()
        stop.group_mask = 0
        stop.remain_valid_millisecs = 100
        req = Land.Request()
        req.group_mask = 0
        req.height = float(height)
        req.duration.sec = int(duration_s)
        req.duration.nanosec = int((duration_s - int(duration_s)) * 1e9)
        self._notify_cli.call_async(stop).add_done_callback(lambda _f: self._land_cli.call_async(req))
        return True
    
    def send_velocity(self, p_meas, v_des, fixed_z: float, yaw: float = 0.0) -> None:
        now = self._node.get_clock().now()
        if self._last_time is None:
            dt = self._dt
        else:
            dt = (now - self._last_time).nanoseconds / 1e9
            if dt <= 0.0:
                return
        self._last_time = now

        p_xy = np.asarray(p_meas, dtype=float)[:2]
        v_des_xy = np.asarray(v_des, dtype=float)[:2]

        if self._p_ref is None or self._v_ref is None:
            self._p_ref = p_xy.copy()
            self._v_ref = np.zeros(2)

        v_ref = self._clip_norm(v_des_xy, self._max_velocity)
        previous_v_ref = self._v_ref

        a_xy = (v_ref - previous_v_ref) / dt if dt > 0.0 else np.zeros(2)
        a_xy = self._clip_norm(a_xy, self._max_acceleration)
        v_ref = previous_v_ref + a_xy * dt

        p_ref = self._p_ref + previous_v_ref * dt + 0.5 * a_xy * dt**2
        self._p_ref, self._v_ref = p_ref, v_ref

        position = np.array([p_ref[0], p_ref[1], fixed_z])
        velocity = np.array([v_ref[0], v_ref[1], 0.0])
        acceleration = np.array([a_xy[0], a_xy[1], 0.0])
        self.send_state(position, velocity, acceleration, yaw)

    def send_velocity_world(self, velocity_xy, vz: float = 0.0, yaw_rate: float = 0.0) -> None:
        """Raw velocity command, no position tracking at all -- the
        firmware's own inner loop handles position/altitude internally.
        No p_ref, no integration, no self-consistency to maintain here;
        that's the whole point of this mode. vz=0.0 commands zero climb
        rate, which is NOT the same as holding a fixed altitude -- it's
        just "don't move vertically right now."
        """
        vxy = self._clip_norm(np.asarray(velocity_xy, dtype=float), self._max_velocity)

        msg = VelocityWorld()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.vel.x, msg.vel.y, msg.vel.z = float(vxy[0]), float(vxy[1]), float(vz)
        msg.yaw_rate = float(yaw_rate)
        self._velocity_world_publisher.publish(msg)

    def send_accel(self, p_meas, a_des, yaw: float = 0.0) -> None:
        p = np.asarray(p_meas, dtype=float)
        a = np.asarray(a_des, dtype=float)

        if self._p_ref is None or self._v_ref is None:
            self._p_ref = p.copy()
            self._v_ref = np.zeros(3)

        a = self._clip_norm(a, self._max_acceleration)
        previous_v_ref = self._v_ref
        v_ref = self._clip_norm(previous_v_ref + a * self._dt, self._max_velocity)

        if self._dt > 0.0:
            a = (v_ref - previous_v_ref) / self._dt

        p_ref = self._p_ref + previous_v_ref * self._dt + 0.5 * a * self._dt**2

        self._p_ref, self._v_ref = p_ref, v_ref
        self.send_state(p_ref, v_ref, a, yaw)

    def send_state(self, position, velocity, acceleration, yaw: float = 0.0) -> None:
        msg = FullState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(position[0]), float(position[1]), float(position[2])
        )
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (
            float(velocity[0]), float(velocity[1]), float(velocity[2])
        )
        msg.acc.x, msg.acc.y, msg.acc.z = (
            float(acceleration[0]), float(acceleration[1]), float(acceleration[2])
        )
        self._publisher.publish(msg)

    @staticmethod
    def _clip_norm(vector, max_norm):
        if max_norm is None:
            return vector
        norm = np.linalg.norm(vector)
        if norm <= max_norm or norm == 0.0:
            return vector
        return vector * (max_norm / norm)