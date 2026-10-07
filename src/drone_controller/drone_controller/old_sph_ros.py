"""ROS glue shared by plain_sph.py and swarm_sph.py: ROS parameters -> SphCore.

Everything that is not ROS lives in sph_core.py / sph_diag.py so it can run offline.
"""

from rclpy.time import Time

from drone_controller import sph_core
from drone_controller.sph_core import COMMON_DEFAULTS, LOG_DIR  # noqa: F401  (re-exported)


def read_params(node, overrides=None):
    defaults = dict(COMMON_DEFAULTS)
    defaults.update(overrides or {})
    cfg = {}
    for key, default in defaults.items():
        node.declare_parameter(key, default)
        cfg[key] = node.get_parameter(key).value
    return cfg


def build_core(node, cfg, egos_from_names, density_on_rx, tag, write_meta=True):
    def echo(level, text):
        lg = node.get_logger()
        (lg.error if level == 'ERROR' else lg.warn)(text)

    return sph_core.build_core(cfg, egos_from_names, density_on_rx, tag,
                               echo=echo, write_meta=write_meta)


def header_latency_ms(node, msg):
    """now - msg.header.stamp in ms, or None if the message has no usable stamp."""
    try:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        return (node.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e6
    except Exception:
        return None
