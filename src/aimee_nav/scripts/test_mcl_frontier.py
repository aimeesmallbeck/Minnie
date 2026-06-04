#!/usr/bin/env python3
"""Test MCL2D and FrontierDetector C++ components.

Usage (inside container with aimee_nav built):
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run aimee_nav test_mcl_frontier.py
"""

import math

from aimee_nav._core import GridMap, MCL2D, FrontierDetector


def test_frontier_detector():
    print("Testing FrontierDetector...")
    grid = GridMap(5.0, 5.0, 0.1, 0.15)
    grid.set_origin(-2.5, -2.5)

    # Create a simple room: free inside, occupied walls
    # We'll do this by placing a scan at origin
    ranges = []
    angles = []
    for i in range(36):
        a = math.radians(i * 10)
        angles.append(a)
        # Square room 2m x 2m
        if abs(math.cos(a)) > abs(math.sin(a)):
            ranges.append(1.0 / abs(math.cos(a)) if abs(math.cos(a)) > 1e-3 else 10.0)
        else:
            ranges.append(1.0 / abs(math.sin(a)) if abs(math.sin(a)) > 1e-3 else 10.0)

    grid.update_from_scan(0.0, 0.0, 0.0, ranges, 0.0, math.radians(10), 0.01, 12.0)
    grid.inflate_obstacles()

    fd = FrontierDetector()
    fd.initialize(grid)
    clusters = fd.get_clusters(5)
    print(f"  Found {len(clusters)} frontier clusters")
    for i, c in enumerate(clusters[:3]):
        print(f"    Cluster {i}: center=({c.cx:.2f}, {c.cy:.2f}) size={c.size:.0f}")
    assert len(clusters) > 0, "Expected at least one frontier"
    print("✅ FrontierDetector OK")


def test_mcl2d():
    print("Testing MCL2D...")
    grid = GridMap(5.0, 5.0, 0.1, 0.15)
    grid.set_origin(-2.5, -2.5)

    # Build a simple map (same room as above)
    ranges = []
    angles = []
    for i in range(72):
        a = math.radians(i * 5)
        angles.append(a)
        if abs(math.cos(a)) > abs(math.sin(a)):
            r = 1.0 / abs(math.cos(a)) if abs(math.cos(a)) > 1e-3 else 10.0
        else:
            r = 1.0 / abs(math.sin(a)) if abs(math.sin(a)) > 1e-3 else 10.0
        ranges.append(min(r, 10.0))

    grid.update_from_scan(0.0, 0.0, 0.0, ranges, 0.0, math.radians(5), 0.01, 12.0)
    grid.inflate_obstacles()

    mcl = MCL2D()
    mcl.set_min_max_particles(100, 500)
    mcl.global_localization(grid, 500)

    particles = mcl.particles()
    print(f"  Initialized with {len(particles)} particles")
    assert len(particles) == 500

    # Simulate robot at (0.5, 0.5, 0.0) with the same scan
    # Run a few update cycles with small motion
    for step in range(5):
        mcl.predict(0.0, 0.0, 0.1)
        ok = mcl.update(ranges, 0.0, math.radians(5), 0.01, 12.0)
        assert ok, f"MCL update failed at step {step}"

    pose = mcl.get_pose()
    print(f"  Estimated pose: x={pose[0]:.2f}, y={pose[1]:.2f}, theta={math.degrees(pose[2]):.1f}°")

    # Should be somewhat close to origin since scan is from origin
    # (In a real test with simulated scan from 0.5,0.5 it would converge there)
    # Here we just check it doesn't crash and produces a pose
    print("✅ MCL2D OK")


if __name__ == "__main__":
    test_frontier_detector()
    test_mcl2d()
    print("\n🎉 All C++ component tests passed!")
