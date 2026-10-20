"""Run ALL drones' SPH controllers in a single swarm_sph node.

Usage:
  ros2 launch drone_controller swarm_sph.launch.py total_robots:=5 R:=0.8 f_k:=3.0

Same launch args and defaults as plain_sph.launch.py; only the process layout
differs (one node for the whole swarm instead of one node per drone).
"""

import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FLOAT_ARGS = {
    'fixed_z': 0.5,
    'particle_mass': 3000.0,
    'smoothing_h': 3.0,
    'rho_0': 1000.0,
    'gamma': 1.0,
    'alpha': 1.0,
    'beta': 2.0,
    'zetta_extra': 50.0,
    'xc': 2.0,
    'yc': 2.0,
    'R': 0.5,
    'f_beta': 1.0,
    'f_k': 2.0,
    'f_kt': 0.0,
    'f_g1': 0.0,
    # --- diagnostics / test switches (defaults == old behaviour) ---
    'sph_gain': 1.0,
    'control_period': 0.05,
    'dt_model': 0.0,
    'test_amp': 0.15,
    'test_period': 4.0,
    'test_delay': 3.0,
    'test_duration': 60.0,
    'max_velocity': 0.5,
    'keepalive_s': 0.1,
    'poll_period': 0.005,
    'fence_xmin': float('nan'), 'fence_xmax': float('nan'),   # m; nan = off (REAL drones: set these)
    'fence_ymin': float('nan'), 'fence_ymax': float('nan'),
    'state_timeout_s': 0.0,
    # --- 3D mode ---
    'dim': 2.0, 'zc': 1.0, 'max_vz': 0.25, 'z_min': 0.25, 'z_max': 2.5,
    'z_margin': 0.3, 'z_wall_k': 2.0, 'z_stretch': 1.0,
    'obs_sigma': 10.0,
    'test_side': 1.0,
}

# name -> default (strings)
STR_ARGS = {
    'potential': 'default',      # default | circle | sphere | quartic
    'stale_policy': 'predict',   # predict | hold | skip
    'start_gate': 'none',        # none | all_known | flight
    'test_profile': 'flight',      # none | zero | step | square | sine | chirp
    'test_axis': 'x',
    'run_id': '',                # '' = timestamp made here, ONE value shared by every node
    'run_label': 'run',
    'git_rev': 'unknown',
    'sim_clock': 'wall',          # wall | gz  (gz = tick on Gazebo simulated time)
    'gz_stats_topic': '',
    'platform': 'sim',           # sim | real
    'log_dir': '/root/dev_ws/src/logs',   # where logs/runs/<run_id>/ is written
    'obstacles': 'none',
}


def _nodes(context):
    shared = {k: float(LaunchConfiguration(k).perform(context)) for k in FLOAT_ARGS}
    shared.update({k: LaunchConfiguration(k).perform(context) for k in STR_ARGS})
    if not shared['run_id']:
        shared['run_id'] = f"{time.strftime('%Y%m%d_%H%M%S')}_{shared['run_label']}"
    return [
        Node(
            package='drone_controller',
            executable='swarm_sph',
            name='swarm_sph',
            output='screen',
            parameters=[{
                'robot_prefix': LaunchConfiguration('robot_prefix').perform(context),
                'total_robots': int(LaunchConfiguration('total_robots').perform(context)),
                **shared,
            }],
        )
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('total_robots', default_value='2'),
        DeclareLaunchArgument('robot_prefix', default_value='cf_'),
    ]
    args += [DeclareLaunchArgument(k, default_value=str(v)) for k, v in FLOAT_ARGS.items()]
    args += [DeclareLaunchArgument(k, default_value=v) for k, v in STR_ARGS.items()]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])