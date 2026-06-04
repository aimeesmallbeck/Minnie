#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL
#
# SPDX-License-Identifier: MPL-2.0

"""
Obstacle avoidance using sector analysis and Virtual Force Field (VFF).

Provides both:
1. Sector-based clearance analysis (front, front-left, front-right, etc.)
2. VFF-based steering computation for goal-directed navigation
3. Emergency behaviors when surrounded
"""

import math
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class SectorClearance:
    name: str
    angle_min: float   # degrees
    angle_max: float   # degrees
    min_distance: float
    avg_distance: float
    is_blocked: bool


class ObstacleAvoidance:
    """
    Reactive obstacle avoidance for 2D lidar scans.
    """

    def __init__(
        self,
        safety_distance_m: float = 0.35,
        min_clearance_m: float = 0.20,
        emergency_reverse_time_s: float = 0.5,
    ) -> None:
        self.safety_distance = safety_distance_m
        self.min_clearance = min_clearance_m
        self.emergency_reverse_time = emergency_reverse_time_s

        self._last_emergency_time = 0.0
        self._emergency_state = 'normal'  # normal, reversing, spinning

    # ------------------------------------------------------------------
    # Sector analysis
    # ------------------------------------------------------------------

    def analyze_sectors(
        self,
        points: List[Tuple[float, float, int]],  # (angle_deg, distance_m, intensity)
    ) -> List[SectorClearance]:
        """
        Analyze clearance in predefined sectors.
        """
        sectors = [
            ('front', -30.0, 30.0),
            ('front_left', 30.0, 90.0),
            ('front_right', -90.0, -30.0),
            ('left', 90.0, 150.0),
            ('right', -150.0, -90.0),
            ('rear', 150.0, 210.0),
        ]

        results = []
        for name, a_min, a_max in sectors:
            distances = []
            for angle, dist, _ in points:
                # Normalize angle to [-180, 180] for comparison
                a = ((angle + 180) % 360) - 180
                if a_min <= a <= a_max:
                    if 0 < dist < float('inf'):
                        distances.append(dist)

            if distances:
                min_d = min(distances)
                avg_d = sum(distances) / len(distances)
            else:
                min_d = float('inf')
                avg_d = float('inf')

            results.append(SectorClearance(
                name=name,
                angle_min=a_min,
                angle_max=a_max,
                min_distance=min_d,
                avg_distance=avg_d,
                is_blocked=min_d < self.safety_distance,
            ))

        return results

    def find_best_direction(
        self,
        points: List[Tuple[float, float, int]],
        preferred_angle_deg: float = 0.0,
    ) -> float:
        """
        Find the direction with maximum clearance, biased toward preferred angle.
        Returns the best angle in degrees.
        """
        best_angle = preferred_angle_deg
        best_score = -float('inf')

        for angle, dist, _ in points:
            if dist <= 0 or math.isinf(dist):
                continue

            # Score = clearance * angular proximity bonus
            angle_diff = abs(((angle - preferred_angle_deg + 180) % 360) - 180)
            proximity_bonus = max(0.0, 1.0 - angle_diff / 90.0)
            score = dist * (1.0 + proximity_bonus)

            if score > best_score:
                best_score = score
                best_angle = angle

        return best_angle

    # ------------------------------------------------------------------
    # Virtual Force Field
    # ------------------------------------------------------------------

    def compute_vff(
        self,
        points: List[Tuple[float, float, int]],
        goal_angle_deg: float,
        goal_distance_m: float,
        attract_gain: float = 1.0,
        repulse_gain: float = 5.0,
        repulse_range_m: float = 2.0,
    ) -> Tuple[float, float]:
        """
        Compute attractive and repulsive forces.

        Returns:
            (fx, fy): resultant force vector in robot frame
                x = forward, y = left
        """
        # Attractive force toward goal
        goal_rad = math.radians(goal_angle_deg)
        f_attract_x = attract_gain * goal_distance_m * math.cos(goal_rad)
        f_attract_y = attract_gain * goal_distance_m * math.sin(goal_rad)

        # Repulsive force from obstacles
        f_repulse_x = 0.0
        f_repulse_y = 0.0

        for angle_deg, dist, _ in points:
            if dist <= 0 or math.isinf(dist) or dist > repulse_range_m:
                continue

            angle_rad = math.radians(angle_deg)
            # Repulsion strength inversely proportional to distance squared
            strength = repulse_gain * (1.0 / dist - 1.0 / repulse_range_m) / (dist * dist)

            # Push away from obstacle
            f_repulse_x -= strength * math.cos(angle_rad)
            f_repulse_y -= strength * math.sin(angle_rad)

        fx = f_attract_x + f_repulse_x
        fy = f_attract_y + f_repulse_y

        return fx, fy

    def vff_to_velocity(
        self,
        fx: float,
        fy: float,
        max_speed: float = 0.5,
        max_angular: float = 1.5,
    ) -> Tuple[float, float]:
        """
        Convert VFF force vector to (linear_x, angular_z) velocity commands.
        """
        # Desired heading from force vector
        desired_heading = math.atan2(fy, fx)

        # Speed proportional to forward component
        linear_x = max(0.0, fx) * max_speed
        linear_x = min(max_speed, linear_x)

        # Angular velocity proportional to heading error
        angular_z = desired_heading * max_angular / math.pi
        angular_z = max(-max_angular, min(max_angular, angular_z))

        return linear_x, angular_z

    # ------------------------------------------------------------------
    # Emergency detection
    # ------------------------------------------------------------------

    def check_emergency(
        self,
        sectors: List[SectorClearance],
    ) -> Optional[str]:
        """
        Check if robot is in an emergency situation.

        Returns emergency action string or None.
        """
        front = next((s for s in sectors if s.name == 'front'), None)
        left = next((s for s in sectors if s.name == 'left'), None)
        right = next((s for s in sectors if s.name == 'right'), None)
        rear = next((s for s in sectors if s.name == 'rear'), None)

        if front is None:
            return None

        now = time.time()

        # If just started emergency, allow time to complete
        if self._emergency_state != 'normal' and now - self._last_emergency_time < self.emergency_reverse_time:
            return self._emergency_state

        self._emergency_state = 'normal'

        # Front blocked, both sides blocked → reverse
        if front.is_blocked:
            left_blocked = left is not None and left.is_blocked
            right_blocked = right is not None and right.is_blocked

            if left_blocked and right_blocked:
                self._emergency_state = 'reverse'
                self._last_emergency_time = now
                return 'reverse'

            # Front blocked, one side free → turn toward free side
            if left_blocked and not right_blocked:
                self._emergency_state = 'spin_right'
                self._last_emergency_time = now
                return 'spin_right'
            if right_blocked and not left_blocked:
                self._emergency_state = 'spin_left'
                self._last_emergency_time = now
                return 'spin_left'

            # Front blocked, both sides somewhat clear → sharp turn
            if left and right:
                if left.avg_distance > right.avg_distance:
                    return 'turn_left'
                else:
                    return 'turn_right'

        return None

    def get_emergency_velocity(
        self,
        action: str,
        max_speed: float = 0.5,
        max_angular: float = 1.5,
    ) -> Tuple[float, float]:
        """Return velocity command for an emergency action."""
        if action == 'reverse':
            return -max_speed * 0.5, 0.0
        elif action == 'spin_left':
            return 0.0, max_angular
        elif action == 'spin_right':
            return 0.0, -max_angular
        elif action == 'turn_left':
            return -max_speed * 0.3, max_angular * 0.7
        elif action == 'turn_right':
            return -max_speed * 0.3, -max_angular * 0.7
        return 0.0, 0.0
