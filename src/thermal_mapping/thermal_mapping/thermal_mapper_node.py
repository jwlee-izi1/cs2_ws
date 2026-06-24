"""Central thermal mapper — LAST-WRITE-WINS fusion.

Subscribes to /<drone>/thermal/raw for each drone, projects each ThermalFrame
onto a fixed-extent shared grid, and OVERWRITES previously-stored cell
values with the new reading. The cell also stores the wall-clock time of
that write so a downstream staleness layer can be computed at publish time.

Why last-write-wins (not running mean):
    Under the dynamic fire model the truth changes — a cell that was hot
    five minutes ago and is now cooling shouldn't pull the estimate up
    with old high samples. The simplest honest policy is "newest reading
    wins, between visits the value is frozen and only age grows."

Packet loss (Bucket C visualization, off by default):
    `packet_drop_prob > 0` enables a per-frame Bernoulli drop at receive
    time. On drop, the thermal value is NOT overwritten — last-good data
    stays — but the cells covered by the dropped frame's footprint are
    marked in the `dropped` layer (1.0). A subsequent accepted frame at
    the same footprint clears the marker (same last-write-wins semantic).
    Drone identity is preserved via `ThermalFrame.pose` so the dropped
    footprint matches the drone's CURRENT physical pose at the drop
    instant (not the optimizer's leash r_i^k).

Published layers:
    thermal            — last-observed temperature (degC), NaN until observed
    age_seconds        — seconds since last observation at publish time;
                         NaN until observed; monotonically grows between
                         visits.
    last_update_time   — seconds-since-epoch of the most recent write
                         (header-stamp), NaN until observed. Used for
                         rosbag-replay reference; not for human eyes.
    dropped            — 1.0 if last write attempt at this cell was a
                         dropped packet, 0.0 otherwise. RViz/matplotlib
                         renders 1.0 cells as a white overlay on top of
                         the (possibly stale) thermal layer.

Single-threaded executor: rclpy.spin serializes callbacks so the buffers
need no locks.
"""

import math
import random
import time

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from thermal_mapping_interfaces.msg import ThermalFrame
from visualization_msgs.msg import Marker
from coverage_optimizer_interfaces.msg import Leash

from thermal_mapping.grid_map_helpers import (
    grid_shape,
    make_grid_map,
    make_info,
    world_to_rowcol,
)


class ThermalMapperNode(Node):

    def __init__(self):
        super().__init__('thermal_mapper')

        self.declare_parameter('drone_names', ['cf1', 'cf2', 'cf3', 'cf4'])
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('length_x', 5.0)
        self.declare_parameter('length_y', 5.0)
        self.declare_parameter('resolution', 0.01)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('packet_drop_prob', 0.0)
        self.declare_parameter('drop_seed', 42)
        # Drop mode:
        #   "fixed"      — packet_drop_prob is the constant Bernoulli rate.
        #   "constraint" — drop rate is computed from live Σ c_i·r_i² vs B_op:
        #       drop = clip((sigma - B_op) * CONSTRAINT_SCALE, 0, MAX_DROP)
        #       sigma comes from /coverage/leash (radii) interpreted via c_i.
        #       Real drone radii from /cfN/odom are used if available; otherwise
        #       fall back to the optimizer's commanded radii.
        self.declare_parameter('drop_mode', 'fixed')
        self.declare_parameter('constraint_B_op', 8.15)
        self.declare_parameter('constraint_scale', 0.5)
        self.declare_parameter('constraint_max_drop', 0.8)
        self.declare_parameter('drone_c_coeffs', [1.0, 1.0, 1.0, 1.0])
        # Dropped-cell TTL (seconds). 0 disables decay (legacy behavior:
        # drops persist until the cell is re-observed via a successful
        # frame). >0 auto-clears a dropped cell `dropped_ttl_seconds`
        # after it was marked, so visualizations show "drops happening
        # now" rather than indefinite history.
        self.declare_parameter('dropped_ttl_seconds', 0.0)
        # Publish a TEXT_VIEW_FACING marker with the live Σ c·r² and
        # violation state (RED "VIOLATION" / GREEN "OK") plus the current
        # optimizer round k (counted from /coverage/leash arrivals).
        # Additionally publishes a SPHERE marker above each drone that
        # is currently in violation (visualization-only, doesn't touch
        # the dropped layer or thermal layer). Sphere only appears while
        # the drone's r > sqrt(B_op / Σ_others / c_i) is true.
        self.declare_parameter('publish_status_marker', False)

        drone_names = list(self.get_parameter('drone_names').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        length_x = float(self.get_parameter('length_x').value)
        length_y = float(self.get_parameter('length_y').value)
        resolution = float(self.get_parameter('resolution').value)
        center_xy = (
            float(self.get_parameter('center_x').value),
            float(self.get_parameter('center_y').value),
        )
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.packet_drop_prob = float(self.get_parameter('packet_drop_prob').value)
        drop_seed = int(self.get_parameter('drop_seed').value)
        self.drop_mode = str(self.get_parameter('drop_mode').value).lower()
        self.constraint_B_op = float(self.get_parameter('constraint_B_op').value)
        self.constraint_scale = float(self.get_parameter('constraint_scale').value)
        self.constraint_max_drop = float(self.get_parameter('constraint_max_drop').value)
        self._drone_c = {
            n: float(c) for n, c in zip(drone_names, list(self.get_parameter('drone_c_coeffs').value))
        }
        self.dropped_ttl_seconds = float(self.get_parameter('dropped_ttl_seconds').value)
        self.publish_status_marker = bool(self.get_parameter('publish_status_marker').value)
        # Live state for status marker.
        self._round_k = 0
        self._sigma_now = 0.0
        # State used to compute live Σ c·r².
        self._drone_xy = {n: (0.0, 0.0) for n in drone_names}
        self._drone_names = drone_names
        # Computed drop probability that gets re-evaluated per-frame in
        # constraint mode. In fixed mode it's just packet_drop_prob.
        self._current_drop_prob = self.packet_drop_prob
        if not 0.0 <= self.packet_drop_prob <= 1.0:
            raise ValueError(
                f'packet_drop_prob must be in [0, 1]; got {self.packet_drop_prob}'
            )
        if self.drop_mode not in ('fixed', 'constraint'):
            raise ValueError(f"drop_mode must be 'fixed' or 'constraint'; got {self.drop_mode!r}")
        # Per-drone independent RNG streams so cf1's drops don't change cf2's.
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
        # Init to NaN so cells never touched render as "no data" in RViz.
        # Dropped cells become 1.0, successfully-received cells become 0.0.
        self._dropped = np.full((self.n_rows, self.n_cols), np.nan, dtype=np.float32)
        # Per-cell time-of-drop (header-stamp seconds). Used to expire stale
        # drop marks when dropped_ttl_seconds > 0.
        self._dropped_time = np.full((self.n_rows, self.n_cols), np.nan, dtype=np.float32)

        out_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(GridMap, '/thermal_map', out_qos)

        # Status marker (text overhead + per-drone violation sphere).
        if self.publish_status_marker:
            self.status_pub = self.create_publisher(
                Marker, '/coverage/status_text', 10)
            self.sphere_pub = self.create_publisher(
                Marker, '/coverage/violation_sphere', 10)
            # Count optimizer rounds via /coverage/leash arrivals. Use
            # VOLATILE durability to match the optimizer's publisher.
            leash_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
            self.create_subscription(
                Leash, '/coverage/leash',
                lambda m: setattr(self, '_round_k', self._round_k + 1),
                leash_qos)

        for name in drone_names:
            self.create_subscription(
                ThermalFrame,
                f'/{name}/thermal/raw',
                lambda msg, n=name: self._on_thermal(n, msg),
                qos_profile_sensor_data,
            )
            # Subscribe to each drone's odom for constraint-mode drop rate.
            if self.drop_mode == 'constraint':
                from nav_msgs.msg import Odometry
                self.create_subscription(
                    Odometry,
                    f'/{name}/odom',
                    lambda msg, n=name: self._on_odom(n, msg),
                    qos_profile_sensor_data,
                )

        self.create_timer(1.0 / publish_rate, self._on_publish_tick)

        self.get_logger().info(
            f'thermal_mapper (last-write-wins) up: '
            f'{self.n_rows}x{self.n_cols} cells '
            f'({length_x}x{length_y}m @ {resolution}m), drones={drone_names}, '
            f'publish={publish_rate}Hz, drop_mode={self.drop_mode}, '
            f'B_op={self.constraint_B_op} (if constraint mode)'
        )

    def _on_odom(self, drone_name: str, msg) -> None:
        """Track each drone's xy for live Σ c·r² (constraint mode).

        Drop is BINARY: when Σ > B_op the next incoming frames are 100%
        dropped (full data loss); when Σ ≤ B_op no drops. Matches the
        paper's data-loss model — the constraint either holds or breaks.
        """
        self._drone_xy[drone_name] = (msg.pose.pose.position.x,
                                       msg.pose.pose.position.y)
        sigma = 0.0
        for n in self._drone_names:
            x, y = self._drone_xy.get(n, (0.0, 0.0))
            sigma += self._drone_c.get(n, 1.0) * (x * x + y * y)
        # Binary drop: 1.0 (drop all) if Σ > B_op, else 0.0 (no drops).
        self._current_drop_prob = 1.0 if sigma > self.constraint_B_op else 0.0
        self._sigma_now = sigma

    def _on_thermal(self, drone_name: str, msg: ThermalFrame) -> None:
        if msg.width == 0 or msg.height == 0:
            return
        if len(msg.data) != msg.width * msg.height:
            self.get_logger().warning(
                f'{drone_name}: data length {len(msg.data)} != {msg.width}*{msg.height}'
            )
            return
        if msg.footprint_size <= 0.0:
            return

        px = msg.pose.position.x
        py = msg.pose.position.y
        h = float(msg.footprint_size) * 0.5  # half-width of square footprint [m]

        u = (np.arange(msg.width, dtype=np.float32) + 0.5) / msg.width * 2.0 - 1.0
        v = (np.arange(msg.height, dtype=np.float32) + 0.5) / msg.height * 2.0 - 1.0
        gx, gy = np.meshgrid(u, v, indexing='xy')
        xs = px + gx * h
        ys = py + gy * h
        vals = np.asarray(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

        pixel_size = float(msg.footprint_size) / float(msg.width)
        res = self.info.resolution
        pix_half_cells = max(0, int(math.ceil(pixel_size * 0.5 / res - 0.5)))

        s = 2 * pix_half_cells + 1
        d = np.arange(-pix_half_cells, pix_half_cells + 1, dtype=np.int32)
        d_row, d_col = np.meshgrid(d, d, indexing='xy')

        rows0, cols0, _ = world_to_rowcol(xs, ys, self.info)

        # Broadcast pixel centers to their (S, S) cell neighborhoods.
        rr = rows0.reshape(-1, 1, 1) + d_row[None, :, :]
        cc = cols0.reshape(-1, 1, 1) + d_col[None, :, :]
        vv = np.broadcast_to(vals.reshape(-1, 1, 1), rr.shape)

        valid = (rr >= 0) & (rr < self.n_rows) & (cc >= 0) & (cc < self.n_cols)
        if not valid.any():
            self.get_logger().warning(
                f'{drone_name}: pose ({px:.2f}, {py:.2f}) projects entirely '
                f'outside the {self.info.length_x}x{self.info.length_y}m map '
                f'(centered at {self.info.pose.position.x:.2f}, '
                f'{self.info.pose.position.y:.2f})',
                throttle_duration_sec=5.0,
            )
            return

        # Last-write-wins: numpy assignment with duplicate indices keeps the
        # value at the highest flat-index position. Since neighbouring pixels
        # in a single frame are sampled from the same fire instant, the
        # specific tiebreak doesn't affect the result meaningfully.
        rr_v = rr[valid].astype(np.int32)
        cc_v = cc[valid].astype(np.int32)
        vv_v = vv[valid].astype(np.float32)
        stamp_sec = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        # Per-frame Bernoulli drop at receive. Uses the drone's CURRENT pose
        # (already embedded in msg.pose by thermal_sensor_node) — the white
        # wedge lands wherever the drone physically is, not where its leash
        # said it should be.
        # Effective drop probability: fixed mode uses packet_drop_prob;
        # constraint mode uses live Σ c·r² vs B_op (updated in _on_odom).
        effective_prob = (self._current_drop_prob
                          if self.drop_mode == 'constraint'
                          else self.packet_drop_prob)
        if effective_prob > 0.0:
            rng = self._drop_rngs.get(drone_name)
            if rng is None:
                rng = random.Random(hash(drone_name))
                self._drop_rngs[drone_name] = rng
            if rng.random() < effective_prob:
                self._dropped[rr_v, cc_v] = np.float32(1.0)
                self._dropped_time[rr_v, cc_v] = np.float32(stamp_sec)
                return

        self._value[rr_v, cc_v] = vv_v
        self._last_update_time[rr_v, cc_v] = np.float32(stamp_sec)
        # Successful frame CLEARS any prior drop mark — set to NaN so RViz
        # GridMap renders these cells as transparent (thermal layer shows
        # through). Only cells currently marked as dropped (=1.0) render
        # as WHITE overlay.
        self._dropped[rr_v, cc_v] = np.float32(np.nan)
        self._dropped_time[rr_v, cc_v] = np.float32(np.nan)

    def _on_publish_tick(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        now_sec = now_msg.sec + now_msg.nanosec * 1e-9
        age = (np.float32(now_sec) - self._last_update_time)
        # Cells never observed stay NaN; numpy NaN arithmetic already
        # propagates NaN through age, so no explicit mask needed.
        if self.dropped_ttl_seconds > 0.0:
            drop_age = np.float32(now_sec) - self._dropped_time
            expired = np.isfinite(drop_age) & (drop_age > self.dropped_ttl_seconds)
            if expired.any():
                self._dropped[expired] = np.float32(np.nan)
                self._dropped_time[expired] = np.float32(np.nan)
        msg = make_grid_map(
            self.info,
            frame_id=self.frame_id,
            layers={
                'thermal': self._value,
                'age_seconds': age.astype(np.float32),
                'last_update_time': self._last_update_time,
                'dropped': self._dropped,
            },
            basic_layers=['thermal'],
            stamp=now_msg,
        )
        self.pub.publish(msg)

        if self.publish_status_marker:
            self._publish_status_markers(now_msg)

    def _publish_status_markers(self, now_msg):
        violated = self._sigma_now > self.constraint_B_op
        # Small unobtrusive status text in the top-left corner of the
        # arena (above the wedge boundary). White, no color animation —
        # the violation sphere above each drone is the dramatic visual.
        m = Marker()
        m.header.stamp = now_msg
        m.header.frame_id = self.frame_id
        m.ns = 'status'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = -2.3
        m.pose.position.y = 2.3
        m.pose.position.z = 0.6
        m.pose.orientation.w = 1.0
        m.scale.z = 0.15
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0
        m.color.a = 1.0
        t_demo = 2 * self._round_k            # sim seconds (round_rate 0.5 Hz -> t = 2*k); counts UP from 0
        m.text = (f'k={self._round_k}   T={t_demo:.0f}s   '
                  f'Σ={self._sigma_now:.2f} / {self.constraint_B_op:.2f}')
        m.lifetime.sec = 1
        self.status_pub.publish(m)

        # Per-drone violation sphere — only published while the system is
        # violating; appears above each drone so it's visually obvious
        # which drone is currently dropping. Uses DELETEALL first so the
        # marker disappears the instant the violation ends.
        delete = Marker()
        delete.header.stamp = now_msg
        delete.header.frame_id = self.frame_id
        delete.action = Marker.DELETEALL
        self.sphere_pub.publish(delete)
        if violated:
            for i, n in enumerate(self._drone_names):
                x, y = self._drone_xy.get(n, (0.0, 0.0))
                s = Marker()
                s.header.stamp = now_msg
                s.header.frame_id = self.frame_id
                s.ns = 'violation_sphere'
                s.id = i
                s.type = Marker.SPHERE
                s.action = Marker.ADD
                s.pose.position.x = float(x)
                s.pose.position.y = float(y)
                s.pose.position.z = 1.5
                s.pose.orientation.w = 1.0
                s.scale.x = 0.5; s.scale.y = 0.5; s.scale.z = 0.5
                s.color.r = 1.0; s.color.g = 1.0; s.color.b = 1.0
                s.color.a = 1.0
                s.lifetime.sec = 1
                self.sphere_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
