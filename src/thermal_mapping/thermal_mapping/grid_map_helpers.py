"""Helpers for assembling grid_map_msgs/GridMap messages from numpy buffers.

The grid_map convention (Eigen column-major) is non-obvious. This module
hides it behind two functions:

  * make_info(...)              — build a GridMapInfo for a fixed-extent map.
  * world_to_rowcol(x, y, info) — vectorized world (x, y) -> (row, col) cell
                                  index, with `valid` mask for in-bounds.
  * make_grid_map(info, layers) — assemble a GridMap message from a dict of
                                  (nRows, nCols) numpy arrays.

GridMap cell (row=0, col=0) sits at the corner with LARGEST x AND LARGEST y;
row increases as x decreases, col increases as y decreases. Stored buffer is
column-major (.ravel(order='F') of the (nRows, nCols) array).
"""

import numpy as np
from grid_map_msgs.msg import GridMap, GridMapInfo
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def make_info(length_x: float, length_y: float, resolution: float,
              center_xy=(0.0, 0.0)) -> GridMapInfo:
    info = GridMapInfo()
    info.resolution = float(resolution)
    info.length_x = float(length_x)
    info.length_y = float(length_y)
    info.pose.position.x = float(center_xy[0])
    info.pose.position.y = float(center_xy[1])
    info.pose.position.z = 0.0
    info.pose.orientation.w = 1.0
    return info


def grid_shape(info: GridMapInfo) -> tuple[int, int]:
    n_rows = int(round(info.length_x / info.resolution))
    n_cols = int(round(info.length_y / info.resolution))
    return n_rows, n_cols


def world_to_rowcol(x: np.ndarray, y: np.ndarray, info: GridMapInfo):
    """Convert world (x, y) arrays to (row, col) integer index arrays.

    Returns (rows, cols, valid) all of the same shape as the input arrays.
    `valid` is a boolean mask: True where the index is within the grid.
    """
    n_rows, n_cols = grid_shape(info)
    cx = info.pose.position.x
    cy = info.pose.position.y
    res = info.resolution
    rows = np.floor((cx + info.length_x * 0.5 - x) / res).astype(np.int32)
    cols = np.floor((cy + info.length_y * 0.5 - y) / res).astype(np.int32)
    valid = (rows >= 0) & (rows < n_rows) & (cols >= 0) & (cols < n_cols)
    return rows, cols, valid


def make_grid_map(info: GridMapInfo,
                  frame_id: str,
                  layers: dict[str, np.ndarray],
                  basic_layers: list[str] | None = None,
                  stamp=None) -> GridMap:
    """Build a GridMap message. Each layer array must have shape (nRows, nCols)
    where nRows = round(length_x / resolution), nCols = round(length_y / res).
    """
    n_rows, n_cols = grid_shape(info)
    msg = GridMap()
    msg.info = info
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp
    msg.layers = list(layers.keys())
    msg.basic_layers = list(basic_layers) if basic_layers is not None else []
    msg.data = []
    for name, arr in layers.items():
        if arr.shape != (n_rows, n_cols):
            raise ValueError(
                f'layer {name!r} has shape {arr.shape}; expected {(n_rows, n_cols)}'
            )
        layer_msg = Float32MultiArray()
        col_dim = MultiArrayDimension()
        col_dim.label = 'column_index'
        col_dim.size = n_cols
        col_dim.stride = n_cols * n_rows
        row_dim = MultiArrayDimension()
        row_dim.label = 'row_index'
        row_dim.size = n_rows
        row_dim.stride = n_rows
        layer_msg.layout.dim = [col_dim, row_dim]
        layer_msg.layout.data_offset = 0
        layer_msg.data = arr.astype(np.float32, copy=False).ravel(order='F').tolist()
        msg.data.append(layer_msg)
    msg.outer_start_index = 0
    msg.inner_start_index = 0
    return msg
