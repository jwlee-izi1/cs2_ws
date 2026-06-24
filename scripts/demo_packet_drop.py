"""Headless demo of the mapper-side packet-drop visualization.

What this does:
- Builds the REAL ThermalMapperNode in-process (no ROS spin, no Gazebo).
- Synthesizes ThermalFrames for N drones flying circles around the arena,
  sampling the actual ThermalField (uses the project's thermal_field.yaml).
- Feeds them into ThermalMapperNode._on_thermal at packet_drop_prob > 0.
- Snapshots the thermal + dropped layers and renders matplotlib panels:
    1. Thermal layer alone (what RViz shows on the bottom layer)
    2. Dropped layer alone (the white-overlay layer)
    3. Composite: thermal as background, white overlay where dropped == 1.0
       — this is what a viewer with both GridMap displays enabled sees.
- Saves the figure to /tmp/demo_packet_drop.png so it can be inspected.

Run:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 scripts/demo_packet_drop.py
"""

import math
import os
import random
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node as _Node
from rclpy.parameter import Parameter
from thermal_mapping_interfaces.msg import ThermalFrame

from thermal_mapping.grid_map_helpers import grid_shape, make_info
from thermal_mapping.thermal_field import ThermalField
from thermal_mapping.thermal_mapper_node import ThermalMapperNode


# ---- Demo parameters ----
DRONE_NAMES = ['cf1', 'cf2', 'cf3', 'cf4']
ARENA_RADIUS_M = 1.7         # circle each drone flies
DRONE_ALTITUDE = 1.5
TICK_RATE_HZ = 5.0
SIM_DURATION_S = 60.0        # 5 min compressed into 60 s of synthetic ticks
SIM_TIME_SCALE = 5.0         # 1 wall sec = 5 simulated sec (so fire develops)
SENSOR_RES = 16
FOV_DEG = 45.0
NOISE_SIGMA = 0.5
PACKET_DROP_PROB = 0.25       # 25% drop → expect plenty of white wedges
DROP_SEED = 1234
# Default save location: inside the workspace so the IDE can open the link
# directly. Override with $DEMO_PACKET_DROP_OUT=/path/to/out.png if desired.
OUT_PNG = os.environ.get(
    'DEMO_PACKET_DROP_OUT',
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'exp1', 'paper_figures', 'demo_packet_drop.png',
    ),
)


def _build_mapper(packet_drop_prob: float) -> ThermalMapperNode:
    """Construct the real ThermalMapperNode with parameter overrides."""
    class _DemoMapperNode(ThermalMapperNode):
        def __init__(self):
            _Node.__init__(
                self,
                'thermal_mapper_demo',
                parameter_overrides=[
                    Parameter('drone_names', Parameter.Type.STRING_ARRAY, DRONE_NAMES),
                    Parameter('frame_id', Parameter.Type.STRING, 'world'),
                    Parameter('length_x', Parameter.Type.DOUBLE, 4.0),
                    Parameter('length_y', Parameter.Type.DOUBLE, 4.0),
                    Parameter('resolution', Parameter.Type.DOUBLE, 0.02),
                    Parameter('center_x', Parameter.Type.DOUBLE, 0.0),
                    Parameter('center_y', Parameter.Type.DOUBLE, 0.0),
                    Parameter('publish_rate_hz', Parameter.Type.DOUBLE, 2.0),
                    Parameter('packet_drop_prob', Parameter.Type.DOUBLE, packet_drop_prob),
                    Parameter('drop_seed', Parameter.Type.INTEGER, DROP_SEED),
                ],
            )
            self.declare_parameter('drone_names', DRONE_NAMES)
            self.declare_parameter('frame_id', 'world')
            self.declare_parameter('length_x', 5.0)
            self.declare_parameter('length_y', 5.0)
            self.declare_parameter('resolution', 0.01)
            self.declare_parameter('center_x', 0.0)
            self.declare_parameter('center_y', 0.0)
            self.declare_parameter('publish_rate_hz', 2.0)
            self.declare_parameter('packet_drop_prob', 0.0)
            self.declare_parameter('drop_seed', 42)

            drone_names = list(self.get_parameter('drone_names').value)
            self.frame_id = str(self.get_parameter('frame_id').value)
            length_x = float(self.get_parameter('length_x').value)
            length_y = float(self.get_parameter('length_y').value)
            resolution = float(self.get_parameter('resolution').value)
            center_xy = (
                float(self.get_parameter('center_x').value),
                float(self.get_parameter('center_y').value),
            )
            self.packet_drop_prob = float(self.get_parameter('packet_drop_prob').value)
            drop_seed = int(self.get_parameter('drop_seed').value)
            self._drop_rngs = {
                n: random.Random(drop_seed + i) for i, n in enumerate(drone_names)
            }

            self.info = make_info(length_x, length_y, resolution, center_xy)
            self.n_rows, self.n_cols = grid_shape(self.info)
            nan = np.float32(np.nan)
            self._value = np.full((self.n_rows, self.n_cols), nan, dtype=np.float32)
            self._last_update_time = np.full(
                (self.n_rows, self.n_cols), nan, dtype=np.float32
            )
            self._dropped = np.zeros((self.n_rows, self.n_cols), dtype=np.float32)

    return _DemoMapperNode()


def _make_frame(px: float, py: float, pz: float, t_sec: float,
                field: ThermalField, fov_deg: float, res: int,
                noise_sigma: float, rng: np.random.Generator) -> ThermalFrame:
    """Synthesize a ThermalFrame as thermal_sensor_node would."""
    tan_half_fov = math.tan(math.radians(fov_deg) * 0.5)
    h = pz * tan_half_fov
    u = (np.arange(res, dtype=np.float32) + 0.5) / res * 2.0 - 1.0
    gx, gy = np.meshgrid(u, u, indexing='xy')
    xs = px + gx * h
    ys = py + gy * h
    truth = field.evaluate(xs, ys, t_sec)
    noise = rng.normal(0.0, noise_sigma, size=truth.shape).astype(np.float32)
    sample = truth + noise

    msg = ThermalFrame()
    t_int = int(t_sec)
    t_frac_ns = int((t_sec - t_int) * 1e9)
    msg.header.stamp.sec = t_int
    msg.header.stamp.nanosec = t_frac_ns
    msg.header.frame_id = 'world'
    msg.pose.position.x = float(px)
    msg.pose.position.y = float(py)
    msg.pose.position.z = float(pz)
    msg.pose.orientation.w = 1.0
    msg.footprint_size = float(2.0 * h)
    msg.width = res
    msg.height = res
    msg.data = sample.ravel().tolist()
    return msg


def main():
    rclpy.init()

    pkg_share = get_package_share_directory('thermal_mapping')
    field_yaml = os.path.join(pkg_share, 'config', 'thermal_field.yaml')
    field = ThermalField.from_yaml(field_yaml)

    mapper = _build_mapper(PACKET_DROP_PROB)
    rng = np.random.default_rng(42)
    n_ticks = int(SIM_DURATION_S * TICK_RATE_HZ)

    sectors_deg = {  # heading angle (radians) per drone sector midpoint
        'cf1': 0.0,
        'cf2': math.pi / 2,
        'cf3': math.pi,
        'cf4': 3 * math.pi / 2,
    }
    n_drones_in = {n: 0 for n in DRONE_NAMES}

    print(f'Simulating {SIM_DURATION_S}s ({n_ticks} ticks @ {TICK_RATE_HZ} Hz)'
          f' × {len(DRONE_NAMES)} drones, packet_drop_prob={PACKET_DROP_PROB}')

    for k in range(n_ticks):
        wall_t = k / TICK_RATE_HZ
        sim_t = wall_t * SIM_TIME_SCALE
        # Each drone orbits the arena at slightly varying radius (so the
        # footprint covers different ground each rotation — more cells touched).
        for i, name in enumerate(DRONE_NAMES):
            angle = sectors_deg[name] + 0.05 * sim_t
            r = ARENA_RADIUS_M * (0.7 + 0.2 * math.sin(0.07 * sim_t + i))
            px = r * math.cos(angle)
            py = r * math.sin(angle)
            msg = _make_frame(
                px=px, py=py, pz=DRONE_ALTITUDE, t_sec=sim_t,
                field=field, fov_deg=FOV_DEG, res=SENSOR_RES,
                noise_sigma=NOISE_SIGMA, rng=rng,
            )
            mapper._on_thermal(name, msg)
            n_drones_in[name] += 1

    thermal = mapper._value
    dropped = mapper._dropped
    n_thermal_cells = int(np.sum(~np.isnan(thermal)))
    n_dropped_cells = int(np.sum(dropped > 0.5))
    total_cells = thermal.size
    print(f'Map: {mapper.n_rows}x{mapper.n_cols} = {total_cells} cells, '
          f'resolution={mapper.info.resolution:.3f} m')
    print(f'Thermal-written cells: {n_thermal_cells} '
          f'({100.0 * n_thermal_cells / total_cells:.1f}%)')
    print(f'Dropped-marked cells:  {n_dropped_cells} '
          f'({100.0 * n_dropped_cells / total_cells:.1f}%)')
    print(f'Frames per drone: {n_drones_in}')

    # World axes — grid is column-major (Eigen). Plot using imshow with extent.
    half_x = mapper.info.length_x * 0.5
    half_y = mapper.info.length_y * 0.5
    extent = (
        mapper.info.pose.position.y - half_y,
        mapper.info.pose.position.y + half_y,
        mapper.info.pose.position.x - half_x,
        mapper.info.pose.position.x + half_x,
    )

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    cmap_thermal = plt.cm.jet.copy()
    cmap_thermal.set_bad((0.2, 0.2, 0.2))  # NaN → dark gray
    th_plot = axes[0].imshow(thermal, cmap=cmap_thermal, vmin=20, vmax=70,
                             extent=extent, origin='upper')
    axes[0].set_title(f'Thermal layer (RViz lower display)')
    axes[0].set_xlabel('y [m]'); axes[0].set_ylabel('x [m]')
    plt.colorbar(th_plot, ax=axes[0], label='°C')

    drop_masked = np.where(dropped > 0.5, 1.0, np.nan)
    cmap_drop = plt.cm.gray.copy()
    cmap_drop.set_bad((0.0, 0.0, 0.0, 0.0))
    axes[1].imshow(drop_masked, cmap='gray', vmin=0, vmax=1,
                   extent=extent, origin='upper')
    axes[1].set_facecolor((0.2, 0.2, 0.2))
    axes[1].set_title(f'Dropped layer (RViz overlay) — {n_dropped_cells} cells')
    axes[1].set_xlabel('y [m]'); axes[1].set_ylabel('x [m]')

    axes[2].imshow(thermal, cmap=cmap_thermal, vmin=20, vmax=70,
                   extent=extent, origin='upper')
    # White overlay where dropped == 1
    white_overlay = np.zeros((*dropped.shape, 4), dtype=np.float32)
    white_overlay[..., :3] = 1.0  # white
    white_overlay[..., 3] = np.where(dropped > 0.5, 1.0, 0.0)  # alpha
    axes[2].imshow(white_overlay, extent=extent, origin='upper')
    axes[2].set_title(f'Composite (what RViz shows with both layers)')
    axes[2].set_xlabel('y [m]'); axes[2].set_ylabel('x [m]')

    fig.suptitle(
        f'mapper-side packet drop demo — drop_prob={PACKET_DROP_PROB}, '
        f'{SIM_DURATION_S:.0f}s × {len(DRONE_NAMES)} drones',
        fontsize=13,
    )
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=120)
    print(f'\nSaved → {OUT_PNG}')

    mapper.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
