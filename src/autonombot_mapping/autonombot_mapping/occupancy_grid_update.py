"""Log-odds occupancy updates with inverse sensor model (laser-style ray casting)."""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np

PRIOR_PROB = 0.5
PRIOR_LOG_ODDS = math.log(PRIOR_PROB / (1.0 - PRIOR_PROB))


def prob2logodds(p: float) -> float:
    p = min(max(p, 1e-6), 1.0 - 1e-6)
    return math.log(p / (1.0 - p))


def logodds2prob(log_odds: float) -> float:
    try:
        return 1.0 - (1.0 / (1.0 + math.exp(log_odds)))
    except OverflowError:
        return 1.0 if log_odds > 0 else 0.0


def logodds_to_occupancy_value(
    log_odds: float,
    free_thresh: float = 0.35,
    occupied_thresh: float = 0.65,
) -> int:
    """Convert log-odds to nav_msgs OccupancyGrid cell value (-1, 0, 100)."""
    p = logodds2prob(log_odds)
    if p > occupied_thresh:
        return 100
    if p < free_thresh:
        return 0
    return -1


def logodds_grid_to_occupancy(
    log_odds: np.ndarray,
    free_thresh: float = 0.35,
    occupied_thresh: float = 0.65,
) -> np.ndarray:
    out = np.full(log_odds.shape, -1, dtype=np.int8)
    occ_lo = prob2logodds(occupied_thresh)
    free_hi = prob2logodds(free_thresh)
    out[log_odds > occ_lo] = 100
    out[log_odds < free_hi] = 0
    return out


def bresenham_cells(ix0: int, iy0: int, ix1: int, iy1: int) -> List[Tuple[int, int]]:
    """Grid cells along the line from (ix0, iy0) to (ix1, iy1), inclusive."""
    cells: List[Tuple[int, int]] = []
    dx = abs(ix1 - ix0)
    dy = abs(iy1 - iy0)
    x, y = ix0, iy0
    x_inc = 1 if ix0 < ix1 else -1
    y_inc = 1 if iy0 < iy1 else -1
    error = dx - dy

    while True:
        cells.append((x, y))
        if x == ix1 and y == iy1:
            break
        e2 = 2 * error
        if e2 > -dy:
            error -= dy
            x += x_inc
        if e2 < dx:
            error += dx
            y += y_inc
    return cells


def apply_inverse_sensor_model(
    log_odds: np.ndarray,
    robot_ix: int,
    robot_iy: int,
    hit_ix: int,
    hit_iy: int,
    *,
    occ_prob: float = 0.9,
    free_prob: float = 0.35,
    log_odds_min: float = -20.0,
    log_odds_max: float = 20.0,
) -> None:
    """
    Laser-style update: free along ray, occupied at endpoint.
    log_odds shape is (height, width); indexing is [iy, ix].
    """
    height, width = log_odds.shape
    if not (0 <= robot_ix < width and 0 <= robot_iy < height):
        return
    hit_ix = max(0, min(width - 1, hit_ix))
    hit_iy = max(0, min(height - 1, hit_iy))

    free_delta = prob2logodds(free_prob) - PRIOR_LOG_ODDS
    occ_delta = prob2logodds(occ_prob) - PRIOR_LOG_ODDS

    line = bresenham_cells(robot_ix, robot_iy, hit_ix, hit_iy)
    for ix, iy in line[:-1]:
        if 0 <= ix < width and 0 <= iy < height:
            log_odds[iy, ix] = np.clip(
                log_odds[iy, ix] + free_delta, log_odds_min, log_odds_max
            )

    end_ix, end_iy = line[-1]
    if 0 <= end_ix < width and 0 <= end_iy < height:
        log_odds[end_iy, end_ix] = np.clip(
            log_odds[end_iy, end_ix] + occ_delta, log_odds_min, log_odds_max
        )


def init_log_odds_from_grid(grid: np.ndarray) -> np.ndarray:
    """Initialize log-odds from int8 occupancy (-1, 0, 100)."""
    log_odds = np.full(grid.shape, PRIOR_LOG_ODDS, dtype=np.float32)
    log_odds[grid == 0] = prob2logodds(0.1)
    log_odds[grid == 100] = prob2logodds(0.9)
    return log_odds
