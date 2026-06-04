#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL
#
# SPDX-License-Identifier: MPL-2.0

"""
ExploreEngine — Enhanced frontier-based autonomous exploration.

Features:
  - C++ frontier detection (periodic full scan, fast)
  - Safe goal generation with obstacle/edge margin
  - Information-gain + safety + alignment scoring
  - Bootstrap 360° spin on empty map
  - Completion detection with auto-save
"""

import math
import random
import time
from typing import List, Tuple, Optional, Dict

import numpy as np


class ExploreEngine:
    """Enhanced exploration logic for AimeeNav."""

    def __init__(
        self,
        frontier_detector,
        safety_margin_m: float = 0.40,
        info_gain_radius_m: float = 2.0,
        min_frontier_size_m: float = 0.25,
        completion_timeout_s: float = 60.0,
        enable_bootstrap_spin: bool = True,
        scoring_weights: Optional[Dict[str, float]] = None,
        logger=None,
    ) -> None:
        self._detector = frontier_detector
        self._safety_margin = safety_margin_m
        self._info_gain_radius = info_gain_radius_m
        self._min_frontier_size = int(round(min_frontier_size_m / 0.05))  # cells @ 5cm
        self._completion_timeout = completion_timeout_s
        self._enable_bootstrap_spin = enable_bootstrap_spin
        self._logger = logger

        self._weights = scoring_weights or {
            "info": 1.0,
            "distance": 0.5,
            "safety": 0.8,
            "alignment": 0.3,
        }

        # State
        self._last_frontier_update = 0.0
        self._frontier_update_interval = 2.0
        self._last_clusters: List[Tuple[float, float, float]] = []  # (cx, cy, size)
        self._visited_goals: set = set()
        self._bootstrap_done = False
        self._exploration_start_time = 0.0
        self._last_new_frontier_time = 0.0
        self._is_complete = False
        self._bootstrap_start_time = 0.0

    def reset(self) -> None:
        self._last_clusters.clear()
        self._visited_goals.clear()
        self._bootstrap_done = False
        self._is_complete = False
        self._exploration_start_time = time.time()
        self._last_new_frontier_time = time.time()

    def is_bootstrap_needed(self, slam_initialized: bool) -> bool:
        """Return True if we need to do a 360° spin before exploring."""
        if not self._enable_bootstrap_spin:
            return False
        if self._bootstrap_done:
            return False
        return slam_initialized

    def start_bootstrap(self) -> None:
        self._bootstrap_start_time = time.time()
        self._log_info("Exploration bootstrap: starting 360° spin")

    def get_bootstrap_velocity(self) -> Tuple[float, float]:
        """Return (linear_x, angular_z) for bootstrap spin."""
        elapsed = time.time() - self._bootstrap_start_time
        if elapsed > 12.0:  # ~360° at 0.5 rad/s
            self._bootstrap_done = True
            self._log_info("Exploration bootstrap: 360° spin complete")
            return 0.0, 0.0
        return 0.0, 0.5

    def update_frontiers(self, global_map) -> List[Tuple[float, float, float]]:
        """Refresh frontier clusters from the global map."""
        now = time.time()
        if now - self._last_frontier_update < self._frontier_update_interval:
            return self._last_clusters

        self._last_frontier_update = now
        try:
            self._detector.initialize(global_map)
            clusters = self._detector.get_clusters(self._min_frontier_size)
            self._last_clusters = [
                (float(c.cx), float(c.cy), float(c.size))
                for c in clusters
            ]
            if self._last_clusters:
                self._last_new_frontier_time = now
        except Exception as e:
            self._log_warn(f"Frontier detection failed: {e}")
        return self._last_clusters

    def select_best_goal(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        global_map,
        local_grid,
    ) -> Optional[Tuple[float, float]]:
        """Select the best exploration goal from current frontiers."""
        clusters = self._last_clusters
        if not clusters:
            return None

        best_goal = None
        best_score = -1.0

        for cx, cy, size in clusters:
            # Generate safe goal
            goal = self._generate_safe_goal(cx, cy, robot_x, robot_y, global_map)
            if goal is None:
                continue
            gx, gy = goal

            # Skip recently visited goals (dedup by rounded coordinates)
            goal_key = (round(gx, 2), round(gy, 2))
            if goal_key in self._visited_goals:
                continue

            # Skip if occupied in local grid
            if hasattr(local_grid, 'is_occupied') and local_grid.is_occupied(gx, gy):
                continue

            score = self._score_goal(
                gx, gy, size, robot_x, robot_y, robot_theta, global_map
            )
            if score > best_score:
                best_score = score
                best_goal = goal

        if best_goal:
            self._visited_goals.add((round(best_goal[0], 2), round(best_goal[1], 2)))
            # Limit history size
            if len(self._visited_goals) > 100:
                self._visited_goals = set(list(self._visited_goals)[-50:])

        return best_goal

    def _generate_safe_goal(
        self,
        frontier_cx: float,
        frontier_cy: float,
        robot_x: float,
        robot_y: float,
        global_map,
    ) -> Optional[Tuple[float, float]]:
        """Offset frontier centroid away from nearest obstacle into open space."""
        dx = robot_x - frontier_cx
        dy = robot_y - frontier_cy
        d = math.hypot(dx, dy)
        if d < 0.01:
            return frontier_cx, frontier_cy

        # Offset toward robot (known free space)
        offset = self._safety_margin
        gx = frontier_cx + (dx / d) * offset
        gy = frontier_cy + (dy / d) * offset

        # Verify goal is free
        ok, gix, giy = global_map.world_to_grid(gx, gy)
        if ok:
            val = global_map.cell(gix, giy)
            if val < 0 or val >= 50:
                # Try smaller offsets
                for frac in [0.5, 0.25, 0.1]:
                    tx = frontier_cx + (dx / d) * offset * frac
                    ty = frontier_cy + (dy / d) * offset * frac
                    ok2, gix2, giy2 = global_map.world_to_grid(tx, ty)
                    if ok2 and global_map.cell(gix2, giy2) == 0:
                        return tx, ty
                return None
        return gx, gy

    def _score_goal(
        self,
        gx: float,
        gy: float,
        size: float,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        global_map,
    ) -> float:
        """Compute utility score for a candidate goal."""
        dist = math.hypot(gx - robot_x, gy - robot_y)

        # Information gain: count unknown cells within radius
        info_gain = self._count_unknown_around(gx, gy, global_map)

        # Safety: average obstacle distance in radius
        safety = self._safety_score(gx, gy, global_map)

        # Alignment: prefer frontiers in front of robot
        angle_to = math.atan2(gy - robot_y, gx - robot_x)
        heading_error = angle_to - robot_theta
        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi
        alignment = max(0.0, math.cos(heading_error))

        w = self._weights
        score = (
            w["info"] * info_gain +
            w["distance"] * (1.0 / (dist + 0.1)) +
            w["safety"] * safety +
            w["alignment"] * alignment
        )
        return score

    def _count_unknown_around(
        self, wx: float, wy: float, global_map
    ) -> float:
        """Count unknown cells within info_gain_radius."""
        r = self._info_gain_radius
        res = global_map.resolution_m()
        steps = max(1, int(r / res))
        count = 0
        total = 0
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                if dx * dx + dy * dy > steps * steps:
                    continue
                x = wx + dx * res
                y = wy + dy * res
                ok, gx, gy = global_map.world_to_grid(x, y)
                if not ok:
                    continue
                total += 1
                val = global_map.cell(gx, gy)
                if val == -1:
                    count += 1
        if total == 0:
            return 0.0
        return count / total

    def _safety_score(self, wx: float, wy: float, global_map) -> float:
        """Return 0.0-1.0 based on obstacle proximity. Higher = safer."""
        check_r = 0.5
        res = global_map.resolution_m()
        steps = max(1, int(check_r / res))
        occupied = 0
        total = 0
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                if dx * dx + dy * dy > steps * steps:
                    continue
                x = wx + dx * res
                y = wy + dy * res
                ok, gx, gy = global_map.world_to_grid(x, y)
                if not ok:
                    continue
                total += 1
                val = global_map.cell(gx, gy)
                if val >= 50:
                    occupied += 1
        if total == 0:
            return 1.0
        ratio = occupied / total
        return max(0.0, 1.0 - ratio * 3.0)

    def check_completion(self) -> bool:
        """Return True if exploration appears complete."""
        if self._is_complete:
            return True
        now = time.time()
        # No frontiers for a while
        if self._last_clusters:
            return False
        if now - self._last_new_frontier_time > self._completion_timeout:
            self._is_complete = True
            self._log_info("Exploration complete: no new frontiers detected")
            return True
        return False

    @property
    def is_complete(self) -> bool:
        return self._is_complete

    def _log_info(self, msg: str) -> None:
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg: str) -> None:
        if self._logger:
            self._logger.warn(msg)
