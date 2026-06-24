"""Full radial coverage demo bringup.

Two-step bringup — timing concerns prevent single-launch composition:

  1. ~/cs2_ws/scripts/thermal_demo.sh up --no-thermal
     (brings up CrazySim + crazyswarm2 + waits for /cfN/takeoff to register)

  2. ros2 launch cf_coverage_planner coverage_demo.launch.py [optimizer:=...]
     (this file: takeoff, thermal sensors, optimizer, planners, streamers, RViz)

Launch args:
  optimizer    : fed_dcsa | constant | sinusoidal     (default: fed_dcsa)
  planner_type : polar_lawnmower | figure8            (default: polar_lawnmower)
                 figure8 uses quadrant_figure8_node — see
                 docs/hw_multi_drone_verification.md.

Loads arena_4drone.yaml as the single source of truth.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_arena(pkg_share):
    yaml_path = os.path.join(pkg_share, 'config', 'arena_4drone.yaml')
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    return cfg, yaml_path


def _takeoff_cmd(altitude, duration_sec=3.0):
    msg = (
        f'{{group_mask: 0, height: {altitude}, '
        f'duration: {{sec: {int(duration_sec)}, nanosec: 0}}}}'
    )
    return ['ros2', 'service', 'call', '/all/takeoff',
            'crazyflie_interfaces/srv/Takeoff', msg]


def _goto_cmd(drone_name, x, y, z, duration_sec=4.0):
    msg = (
        f'{{group_mask: 0, relative: false, '
        f'goal: {{x: {x}, y: {y}, z: {z}}}, yaw: 0.0, '
        f'duration: {{sec: {int(duration_sec)}, nanosec: 0}}}}'
    )
    return ['ros2', 'service', 'call', f'/{drone_name}/go_to',
            'crazyflie_interfaces/srv/GoTo', msg]


def _build_nodes(context, *args, **kwargs):
    pkg_share = get_package_share_directory('cf_coverage_planner')
    cfg, params_file = _load_arena(pkg_share)

    drone_names = list(cfg['drone_names'])
    arena = cfg['arena']
    sectors = cfg['sectors']
    streamer_cfg = cfg['streamer']
    planner_cfg = cfg['planner']
    sensor_cfg = cfg['sensor']
    opt_cfg = cfg['optimizer']

    optimizer = LaunchConfiguration('optimizer').perform(context)
    planner_type = LaunchConfiguration('planner_type').perform(context)
    streamer_type = LaunchConfiguration('streamer_type').perform(context)   # receding_horizon | cubic (rl only)
    rl_checkpoint = LaunchConfiguration('rl_checkpoint').perform(context)
    policy_warmup_sec = float(LaunchConfiguration('policy_warmup_sec').perform(context))
    enable_thermal = LaunchConfiguration('enable_thermal').perform(context).lower() == 'true'
    enable_takeoff = LaunchConfiguration('enable_takeoff').perform(context).lower() == 'true'
    enable_rviz = LaunchConfiguration('enable_rviz').perform(context).lower() == 'true'
    figure8_cfg = cfg.get('figure8', {})
    figure8_traj_csv = os.path.join(
        os.environ.get('HOME', ''), 'cs2_ws',
        figure8_cfg.get('trajectory_csv',
                        'src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/figure8.csv'))

    nodes = []

    # Takeoff (issued once, immediately when launch starts).
    # Skip with `enable_takeoff:=false` if takeoff is being handled externally
    # (e.g., recording-friendly bringup where user issues /all/takeoff manually
    # before launching this for the policy-only phase).
    if enable_takeoff:
        nodes.append(ExecuteProcess(
            cmd=_takeoff_cmd(arena['altitude']),
            output='screen',
        ))

    settle = float(streamer_cfg['takeoff_settle_sec'])
    # Two-phase deferred spawn:
    #   thermal_deferred  → spawns at  settle           (thermal + RViz come up first)
    #   policy_deferred   → spawns at  settle + policy_warmup_sec (optimizer + planner + streamer)
    # With policy_warmup_sec > 0, drones hover near their takeoff (x,y) while thermal
    # captures the central region BEFORE the figure-8 policy starts and walks them outward.
    thermal_deferred = []
    policy_deferred = []

    # Thermal sensors with overridden FOV. → THERMAL phase
    # Skip with `enable_thermal:=false` if thermal pipeline is being launched
    # externally (e.g., recording-friendly bringup where thermal_mapping_demo.launch.py
    # was started before this for early warmup).
    if enable_thermal:
        thermal_pkg = get_package_share_directory('thermal_mapping')
        thermal_params = os.path.join(thermal_pkg, 'config', 'thermal_mapping_params.yaml')
        thermal_field = os.path.join(thermal_pkg, 'config', 'thermal_field.yaml')
        for name in drone_names:
            thermal_deferred.append(Node(
                package='thermal_mapping',
                executable='thermal_sensor_node',
                name=f'thermal_sensor_{name}',
                output='screen',
                parameters=[
                    thermal_params,
                    {'drone_name': name,
                     'field_yaml': thermal_field,
                     'fov_deg': float(sensor_cfg['fov_deg']),
                     'resolution': 16,
                     'rate_hz': 5.0,
                     'min_altitude': 0.2,
                     'noise_sigma': 0.5,
                     'use_sim_time': False},
                ],
            ))
        # When the RL planner is active, enable the constraint-driven packet
        # drop visualization (Bucket C). drop rate scales with Σ c·r² over B_op.
        mapper_params = {
            'drone_names': drone_names,
            'use_sim_time': False,
        }
        # 2026-06-04: extended rl -> (rl, figure8) so the figure-8 fallback demo
        # shows the same live K/T/Σ status label + per-drone violation spheres
        # (drop_mode=constraint computes live Σc·r² from odom over B_op).
        if planner_type in ('rl', 'figure8'):
            mapper_params.update({
                'drop_mode': 'constraint',
                'constraint_B_op': 8.40,   # 2026-06-05 receding-horizon iter18: B=8.0 budget + 0.40 margin.
                                           # RH removes the tracking overshoot -> Fed (Sigma_leash 8.08) holds
                                           # Sigma_drone ~7.9 < 8.40 = CLEAN; Lag (8.81) overshoots -> drops.
                                           # (RH also makes Fed clean at the honest 8.15 -- see plan.)
                'constraint_scale': 2.5,        # bumped 0.5 → 2.5 so small Σ
                                                 # excursions over B_op give
                                                 # visibly white drop regions
                'constraint_max_drop': 0.95,    # near-total drops when Σ is well over B_op
                'drone_c_coeffs': [float(sectors[n]['c']) for n in drone_names],
                # 10 s TTL on dropped marks so the visualization shows
                # "drops happening now" rather than persistent history
                # from early λ-warmup violations.
                'dropped_ttl_seconds': 10.0,
                # Publish overhead status text + per-drone violation
                # spheres for live RViz visibility.
                'publish_status_marker': True,
            })
        thermal_deferred.append(Node(
            package='thermal_mapping',
            executable='thermal_mapper_node',
            name='thermal_mapper',
            output='screen',
            parameters=[
                thermal_params,
                mapper_params,
            ],
        ))
        # Oracle q_i estimator → /coverage/sector_weights. Optimizer subscribes
        # and uses these in place of the static q from arena_4drone.yaml.
        # Rate matches the optimizer's round_rate_hz so each round sees a
        # fresh sample of the moving fire.
        thermal_deferred.append(Node(
            package='thermal_mapping',
            executable='qi_estimator_node',
            name='qi_estimator',
            output='screen',
            parameters=[{
                'field_yaml': thermal_field,
                'arena_yaml': params_file,
                'rate_hz': float(opt_cfg['round_rate_hz']),
                'use_sim_time': False,
            }],
        ))
        # Ground-truth thermal field → /thermal_truth GridMap. Visualization-
        # and rosbag-only; sensors still evaluate T(x,y,t) in-process.
        thermal_deferred.append(Node(
            package='thermal_mapping',
            executable='thermal_ground_truth_node',
            name='thermal_ground_truth',
            output='screen',
            parameters=[
                thermal_params,
                {'field_yaml': thermal_field, 'use_sim_time': False},
            ],
        ))

    # Alias for the rest of the legacy code path — optimizer/planner/streamer get
    # appended to `deferred`, which is the policy_deferred list.
    deferred = policy_deferred

    # Optimizer (switchable).
    if optimizer in ('fed_dcsa', 'lagrangian'):
        algorithm = 'feddcsa' if optimizer == 'fed_dcsa' else 'lagrangian'
        deferred.append(Node(
            package='fed_dcsa',
            executable='radial_coverage_node',
            name='radial_coverage_optimizer',
            output='screen',
            parameters=[{
                'drone_names': drone_names,
                'q_weights': [float(sectors[n]['q']) for n in drone_names],
                'c_coeffs': [float(sectors[n]['c']) for n in drone_names],
                'r_star_per_drone': [float(sectors[n]['r_star']) for n in drone_names],
                'r_max_per_drone': [float(sectors[n]['r_max']) for n in drone_names],
                'budget_B': float(opt_cfg['budget_B']),
                'K': int(opt_cfg['K']),
                'T': int(opt_cfg['T']),
                'c1': float(opt_cfg['c_1']),
                'c2': float(opt_cfg['c_2']),
                'noise_bound': float(opt_cfg['noise_bound']),
                'round_rate_hz': float(opt_cfg['round_rate_hz']),
                'algorithm': algorithm,
                # Graceful demo stop: land all drones + freeze at this many sim
                # seconds (0 = off). round_rate 0.5 Hz → stop_round_k = stop_time_s/2.
                'stop_time_s': float(opt_cfg.get('stop_time_s', 0.0)),
            }],
        ))
    elif optimizer == 'constant':
        deferred.append(Node(
            package='baseline_optimizers',
            executable='constant_leash_node',
            name='constant_leash_optimizer',
            output='screen',
            parameters=[{
                'drone_names': drone_names,
                'radii': [float(sectors[n]['r_star']) for n in drone_names],
                'round_rate_hz': float(opt_cfg['round_rate_hz']),
            }],
        ))
    elif optimizer == 'sinusoidal':
        deferred.append(Node(
            package='baseline_optimizers',
            executable='sinusoidal_leash_node',
            name='sinusoidal_leash_optimizer',
            output='screen',
            parameters=[{
                'drone_names': drone_names,
                'r_max_per_drone': [float(sectors[n]['r_max']) for n in drone_names],
                'period_sec': 20.0,
                'round_rate_hz': float(opt_cfg['round_rate_hz']),
            }],
        ))
    else:
        raise ValueError(f'unknown optimizer: {optimizer}')

    # Per-drone planners and streamers.
    for name in drone_names:
        s = sectors[name]
        if planner_type == 'polar_lawnmower':
            deferred.append(Node(
                package='cf_coverage_planner',
                executable='coverage_planner_node',
                name=f'coverage_planner_{name}',
                output='screen',
                parameters=[{
                    'drone_name': name,
                    'center_xy': list(arena['center_xy']),
                    'altitude': float(arena['altitude']),
                    'phi_mid_deg': float(s['phi_mid_deg']),
                    'phi_half_deg': float(s['phi_half_deg']),
                    'r_star': float(s['r_star']),
                    'r_max': float(s['r_max']),
                    'q': float(s['q']),
                    'c': float(s['c']),
                    'footprint_radius': float(planner_cfg['footprint_radius']),
                    'deadband': float(planner_cfg.get('deadband', 0.10)),
                    'drone_speed': float(streamer_cfg['max_setpoint_velocity']),
                    'num_spokes': int(planner_cfg.get('num_spokes', 5)),
                    'inner_radius': float(planner_cfg.get('inner_radius', 0.5)),
                    'planner_rate_hz': float(planner_cfg['planner_rate_hz']),
                }],
            ))
        elif planner_type == 'figure8':
            quad = figure8_cfg['quadrants'][name]
            deferred.append(Node(
                package='cf_coverage_planner',
                executable='quadrant_figure8_node',
                name=f'quadrant_figure8_{name}',
                output='screen',
                parameters=[{
                    'drone_name': name,
                    'quadrant_sign_x': int(quad['sign_x']),
                    'quadrant_sign_y': int(quad['sign_y']),
                    # CARDINAL sector centerline from the optimizer sectors (cf1=0/E, cf2=90/N,
                    # cf3=180/W, cf4=270/S) — matches the cardinal drone placement + the optimizer.
                    'phi_mid_deg':    float(sectors[name]['phi_mid_deg']),
                    # Tangential leash-arc sweep params (2026-06-04 rewrite).
                    'r_frac':         float(figure8_cfg.get('r_frac', 0.99)),
                    'r_max':          float(figure8_cfg.get('r_max', 1.9)),
                    'sweep_amp_frac': float(figure8_cfg.get('sweep_amp_frac', 0.8)),
                    'phi_half_deg':   float(figure8_cfg.get('phi_half_deg', 45.0)),
                    'sweep_period_s': float(figure8_cfg.get('sweep_period_s', 9.0)),
                    'wobble_w':       float(figure8_cfg.get('wobble_w', 0.0)),
                    'min_leash':      float(figure8_cfg.get('min_leash', 0.0)),
                    'altitude': float(figure8_cfg.get('altitude', arena['altitude'])),
                    'planner_rate_hz': float(planner_cfg['planner_rate_hz']),
                }],
            ))
        elif planner_type == 'rl':
            rl_cfg = cfg.get('rl', {})
            # Launch arg overrides yaml; yaml default is empty so the launch arg
            # is the canonical entry point.
            ckpt = rl_checkpoint or rl_cfg.get('checkpoint_path', '')
            if not ckpt:
                raise ValueError(
                    "planner_type=rl requires either 'rl_checkpoint:=<abs-path>' "
                    "launch arg or 'rl.checkpoint_path' in arena_4drone.yaml")
            deferred.append(Node(
                package='cf_coverage_planner',
                executable='rl_planner_node',
                name=f'rl_planner_{name}',
                output='screen',
                parameters=[{
                    'drone_name': name,
                    'center_xy': list(arena['center_xy']),
                    'altitude': float(arena['altitude']),
                    'phi_mid_deg': float(s['phi_mid_deg']),
                    'phi_half_deg': float(s['phi_half_deg']),
                    'r_max': float(s['r_max']),
                    'min_leash': float(rl_cfg.get('min_leash', 0.4)),
                    'action_slack': float(rl_cfg.get('action_slack', 1.0)),
                    'checkpoint_path': str(ckpt),
                    'planner_rate_hz': float(planner_cfg['planner_rate_hz']),
                    'episode_seconds': float(rl_cfg.get('episode_seconds', 360.0)),
                    'age_crop_size': int(rl_cfg.get('age_crop_size', 32)),
                    'past_action_history': int(rl_cfg.get('past_action_history', 10)),
                    'device': str(rl_cfg.get('device', 'cpu')),
                    'airborne_z_threshold': float(rl_cfg.get('airborne_z_threshold', 0.3)),
                    'debug_log_path': f'/tmp/rl_planner_{name}.log',
                }],
            ))
        else:
            raise ValueError(f'unknown planner_type: {planner_type}')

        # For RL planner: use trajectory_streamer (calls /cfN/go_to per tick).
        # Firmware plans its own polynomial trajectory with bounded acceleration,
        # bypassing the position/velocity PID interaction that was causing
        # crashes with cmd_position streaming.
        # For figure-8 / polar_lawnmower: keep setpoint_streamer (works fine
        # for smooth trajectories that don't trigger PID overshoot).
        streamer_vel = float(streamer_cfg['max_setpoint_velocity'])
        if planner_type == 'rl' and streamer_type == 'receding_horizon':
            # Smooth receding-horizon streamer: polar arc -> no radial overshoot,
            # C2 quintic pieces + current-velocity graft -> no jitter. Tracks the RL
            # policy_target (does NOT generate the sweep). See receding_horizon_traj.py.
            rh = cfg.get('rl', {}).get('streamer', {})
            deferred.append(Node(
                package='cf_coverage_planner',
                executable='receding_horizon_streamer_node',
                name=f'receding_horizon_streamer_{name}',
                output='screen',
                parameters=[{
                    'drone_name': name,
                    'altitude': float(arena['altitude']),
                    'horizon_sec': float(rh.get('horizon_sec', 1.5)),
                    'replan_period': float(rh.get('replan_period', 0.5)),
                    'n_pieces': int(rh.get('n_pieces', 5)),
                    'r_min': float(rh.get('r_min', 0.3)),
                    'target_vel_alpha': float(rh.get('target_vel_alpha', 0.4)),
                    'max_target_speed': float(rh.get('max_target_speed', 1.4)),
                    'leash_margin': float(rh.get('leash_margin', 0.0)),
                    'cruise_speed': float(rh.get('cruise_speed', 1.18)),
                    'decel_horizon': float(rh.get('decel_horizon', 0.15)),
                    'sign_deadband_deg': float(rh.get('sign_deadband_deg', 3.0)),
                    'phi_mid_deg': float(s['phi_mid_deg']),
                    'phi_half_deg': float(s['phi_half_deg']),
                    'sweep_half_deg': float(rh.get('sweep_half_deg', 36.0)),
                }],
            ))
        elif planner_type == 'rl':
            streamer_vel = float(cfg.get('rl', {}).get('streamer_max_velocity', 1.0))
            deferred.append(Node(
                package='cf_coverage_planner',
                executable='polynomial_streamer_node',
                name=f'polynomial_streamer_{name}',
                output='screen',
                parameters=[{
                    'drone_name': name,
                    # Each polynomial segment lasts 1.0s with replan every
                    # 0.5s, so the firmware always has a fresh trajectory
                    # respecting current drone position+velocity boundary
                    # conditions. Cubic polynomial → smooth, no overshoot.
                    'segment_duration': 1.0,
                    'replan_period': 0.5,
                    'altitude': float(arena['altitude']),
                }],
            ))
        else:
            deferred.append(Node(
                package='cf_coverage_planner',
                executable='setpoint_streamer_node',
                name=f'setpoint_streamer_{name}',
                output='screen',
                parameters=[{
                    'drone_name': name,
                    'setpoint_rate_hz': float(streamer_cfg['setpoint_rate_hz']),
                    'max_setpoint_velocity': streamer_vel,
                }],
            ))

    # RViz — comes up with thermal so you can see the central map before the policy starts.
    # Skip with `enable_rviz:=false` if an RViz is already running externally.
    if enable_rviz:
        rviz_config = os.path.join(pkg_share, 'rviz', 'coverage.rviz')
        thermal_deferred.append(Node(
            package='rviz2',
            executable='rviz2',
            name='coverage_rviz',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': False}],
        ))

    # Two-phase spawn: thermal at settle, policy at settle + policy_warmup_sec.
    nodes.append(TimerAction(period=settle, actions=thermal_deferred))
    nodes.append(TimerAction(period=settle + policy_warmup_sec, actions=policy_deferred))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'optimizer',
            default_value='fed_dcsa',
            description='Which optimizer publishes /coverage/leash: fed_dcsa | constant | sinusoidal'),
        DeclareLaunchArgument(
            'planner_type',
            default_value='polar_lawnmower',
            description='Per-drone planner: polar_lawnmower | figure8 | rl'),
        DeclareLaunchArgument(
            'streamer_type',
            default_value='receding_horizon',
            description='RL-path streamer: receding_horizon (smooth polar arc, no overshoot) | '
                        'cubic (legacy polynomial_streamer). Only used when planner_type:=rl.'),
        DeclareLaunchArgument(
            'rl_checkpoint',
            default_value='',
            description='Absolute path to SB3 PPO *.zip (used only when planner_type:=rl). '
                        'Overrides rl.checkpoint_path in arena_4drone.yaml.'),
        DeclareLaunchArgument(
            'policy_warmup_sec',
            default_value='0.0',
            description='Seconds AFTER takeoff_settle to wait before spawning optimizer + planner '
                        '+ streamer. Thermal spawns at takeoff_settle (and runs during the warmup), '
                        'so drones hover near takeoff (x,y) with thermal capturing the central '
                        'region BEFORE the policy walks them outward. Use e.g. 10.0 for the demo arc; '
                        '0.0 keeps the pre-2026-05-23 behaviour where thermal+policy spawn together.'),
        DeclareLaunchArgument(
            'enable_thermal',
            default_value='true',
            description='Spawn thermal_sensor + thermal_mapper. Set false if thermal pipeline '
                        'is already running (recording-friendly bringup).'),
        DeclareLaunchArgument(
            'enable_takeoff',
            default_value='true',
            description='Issue /all/takeoff on launch. Set false if drones are already in the air.'),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Spawn RViz. Set false if an RViz is already running.'),
        OpaqueFunction(function=_build_nodes),
    ])
