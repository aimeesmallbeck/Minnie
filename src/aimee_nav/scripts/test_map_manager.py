#!/usr/bin/env python3
"""Test script for MapManager functionality.

Usage:
    ros2 run aimee_nav test_map_manager.py
"""

import os
import sys
import tempfile
import shutil

# Allow running without full ROS2 environment
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from aimee_nav.map_manager import MapManager


def test_map_manager():
    tmpdir = tempfile.mkdtemp(prefix="aimee_map_test_")
    print(f"Test library dir: {tmpdir}")

    class FakeLogger:
        def info(self, msg): print(f"[INFO] {msg}")
        def warn(self, msg): print(f"[WARN] {msg}")
        def error(self, msg): print(f"[ERROR] {msg}")

    mgr = MapManager(tmpdir, logger=FakeLogger())

    # Test empty list
    maps = mgr.list_maps()
    assert len(maps) == 0, "Expected empty list"
    print("✅ list_maps empty OK")

    # Test save native map (mock objects)
    class FakeGridMap:
        def resolution_m(self): return 0.05
        def width_m(self): return 10.0
        def height_m(self): return 10.0
        def width_cells(self): return 200
        def height_cells(self): return 200
        def origin_x(self): return -5.0
        def origin_y(self): return -5.0
        def data(self):
            return [0] * 40000

    import numpy as np
    class FakeEKF:
        def x(self): return 1.0
        def y(self): return 2.0
        def theta(self): return 0.5
        def covariance(self):
            return np.array([1.0] * 9, dtype=np.float32)

    class FakePoseGraph:
        def keyframes(self): return []
        def constraints(self): return []

    ok = mgr.save_native_map(
        map_id="test_home",
        name="Test Home",
        description="A test map",
        global_map=FakeGridMap(),
        ekf=FakeEKF(),
        pose_graph=FakePoseGraph(),
        waypoints={"kitchen": (1.0, 2.0, 0.0)},
    )
    assert ok, "save_native_map failed"
    print("✅ save_native_map OK")

    maps = mgr.list_maps()
    assert len(maps) == 1
    assert maps[0]["id"] == "test_home"
    print("✅ list_maps after save OK")

    # Test load
    data = mgr.load_native_map("test_home")
    assert data is not None
    assert data["map_id"] == "test_home"
    assert data["resolution"] == 0.05
    print("✅ load_native_map OK")

    # Test waypoints
    wps = mgr.load_waypoints("test_home")
    assert "kitchen" in wps
    print("✅ load_waypoints OK")

    # Test delete
    ok = mgr.delete_map("test_home")
    assert ok
    maps = mgr.list_maps()
    assert len(maps) == 0
    print("✅ delete_map OK")

    # Test import/export with a synthetic PGM
    pgm_path = os.path.join(tmpdir, "test.pgm")
    yaml_path = os.path.join(tmpdir, "test.yaml")
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n10 10\n255\n")
        f.write(bytes([0 if (x + y) % 2 == 0 else 255 for x in range(10) for y in range(10)]))
    with open(yaml_path, "w") as f:
        f.write("image: test.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")

    ok = mgr.import_ros_map("contest_map", pgm_path, yaml_path, name="Contest Map")
    assert ok, "import_ros_map failed"
    print("✅ import_ros_map OK")

    maps = mgr.list_maps()
    assert len(maps) == 1
    assert maps[0]["type"] == "ros_map"
    print("✅ list_maps after import OK")

    # Test export
    export_pgm = os.path.join(tmpdir, "export.pgm")
    export_yaml = os.path.join(tmpdir, "export.yaml")
    ok = mgr.export_ros_map("contest_map", export_pgm, export_yaml)
    assert ok, "export_ros_map failed"
    assert os.path.exists(export_pgm)
    assert os.path.exists(export_yaml)
    print("✅ export_ros_map OK")

    # Cleanup
    shutil.rmtree(tmpdir)
    print("\n🎉 All MapManager tests passed!")


if __name__ == "__main__":
    test_map_manager()
