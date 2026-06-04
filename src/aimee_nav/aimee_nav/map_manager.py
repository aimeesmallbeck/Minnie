#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL
#
# SPDX-License-Identifier: MPL-2.0

"""
MapManager — Multi-map library for AimeeNav.

Manages a structured directory of saved maps, supporting:
  - AimeeNav native maps (full serialized state: grid + EKF + pose graph)
  - ROS standard maps (PGM + YAML import/export)
  - Per-map waypoints
  - Manifest indexing for fast listing

Directory structure:
  ~/aimee_maps/
    manifest.json
    <map_id>/
      map.json          # Native AimeeNav format
      metadata.json
      waypoints.yaml
    <map_id>/
      map.pgm           # ROS format
      map.yaml
      metadata.json
      waypoints.yaml
"""

import base64
import json
import math
import os
import shutil
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


class MapManager:
    """Manages the map library on disk."""

    MANIFEST_FILENAME = "manifest.json"
    NATIVE_MAP_FILENAME = "map.json"
    METADATA_FILENAME = "metadata.json"
    WAYPOINTS_FILENAME = "waypoints.yaml"
    ROS_PGM_FILENAME = "map.pgm"
    ROS_YAML_FILENAME = "map.yaml"

    def __init__(self, library_dir: str, logger=None) -> None:
        self._library_dir = Path(os.path.expanduser(library_dir))
        self._library_dir.mkdir(parents=True, exist_ok=True)
        self._logger = logger
        self._manifest: Dict[str, Any] = {"version": 1, "maps": {}}
        self._load_manifest()

    # ------------------------------------------------------------------
    # Manifest helpers
    # ------------------------------------------------------------------

    def _manifest_path(self) -> Path:
        return self._library_dir / self.MANIFEST_FILENAME

    def _load_manifest(self) -> None:
        path = self._manifest_path()
        if path.exists():
            try:
                with open(path, "r") as f:
                    self._manifest = json.load(f)
                if "maps" not in self._manifest:
                    self._manifest["maps"] = {}
            except Exception as e:
                self._log_warn(f"Corrupt manifest, resetting: {e}")
                self._manifest = {"version": 1, "maps": {}}
        else:
            self._save_manifest()

    def _save_manifest(self) -> None:
        try:
            with open(self._manifest_path(), "w") as f:
                json.dump(self._manifest, f, indent=2)
        except Exception as e:
            self._log_error(f"Failed to write manifest: {e}")

    def _map_dir(self, map_id: str) -> Path:
        return self._library_dir / map_id

    def _log_info(self, msg: str) -> None:
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg: str) -> None:
        if self._logger:
            self._logger.warn(msg)

    def _log_error(self, msg: str) -> None:
        if self._logger:
            self._logger.error(msg)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def list_maps(self) -> List[Dict[str, Any]]:
        """Return a list of metadata dicts for all maps."""
        result = []
        for map_id, meta in self._manifest.get("maps", {}).items():
            entry = dict(meta)
            entry["id"] = map_id
            result.append(entry)
        # Sort by name for consistent ordering
        result.sort(key=lambda x: x.get("name", x.get("id", "")))
        return result

    def get_map(self, map_id: str) -> Optional[Dict[str, Any]]:
        """Get metadata for a specific map."""
        meta = self._manifest.get("maps", {}).get(map_id)
        if meta is None:
            return None
        entry = dict(meta)
        entry["id"] = map_id
        return entry

    def has_map(self, map_id: str) -> bool:
        return map_id in self._manifest.get("maps", {})

    def save_native_map(
        self,
        map_id: str,
        name: str,
        description: str,
        global_map,
        ekf,
        pose_graph,
        waypoints: Optional[Dict[str, Tuple[float, float, float]]] = None,
    ) -> bool:
        """Save current AimeeNav state as a native map."""
        try:
            map_dir = self._map_dir(map_id)
            map_dir.mkdir(parents=True, exist_ok=True)

            # Serialize native map
            native_data = self._serialize_native(global_map, ekf, pose_graph)
            native_path = map_dir / self.NATIVE_MAP_FILENAME
            with open(native_path, "w") as f:
                json.dump(native_data, f, indent=2)

            # Write metadata
            metadata = {
                "name": name or map_id,
                "description": description or "",
                "type": "aimee_nav",
                "created_at": self._now_iso(),
                "updated_at": self._now_iso(),
                "resolution": global_map.resolution_m(),
                "width_m": global_map.width_m(),
                "height_m": global_map.height_m(),
                "origin_x": global_map.origin_x(),
                "origin_y": global_map.origin_y(),
                "has_waypoints": bool(waypoints),
            }
            meta_path = map_dir / self.METADATA_FILENAME
            with open(meta_path, "w") as f:
                json.dump(metadata, f, indent=2)

            # Write waypoints
            if waypoints:
                self._write_waypoints(map_dir, waypoints)

            # Update manifest
            self._manifest["maps"][map_id] = metadata
            self._save_manifest()

            self._log_info(f"Native map saved: {map_id} -> {map_dir}")
            return True
        except Exception as e:
            self._log_error(f"Failed to save native map {map_id}: {e}")
            return False

    def load_native_map(self, map_id: str) -> Optional[Dict[str, Any]]:
        """Load a native AimeeNav map from disk. Returns dict with grid_data, ekf, pose_graph."""
        try:
            map_dir = self._map_dir(map_id)
            native_path = map_dir / self.NATIVE_MAP_FILENAME
            if not native_path.exists():
                self._log_warn(f"Native map file not found: {native_path}")
                return None

            with open(native_path, "r") as f:
                data = json.load(f)

            # Decode grid
            map_data = data["map"]
            grid_bytes = base64.b64decode(map_data["grid_data_b64"])
            grid_list = [b if b < 128 else b - 256 for b in grid_bytes]

            ekf_data = data.get("ekf", {})
            pg_data = data.get("pose_graph", {})

            # Reconstruct pose graph keyframes
            keyframes = []
            for kf_json in pg_data.get("keyframes", []):
                keyframes.append({
                    "x": float(kf_json["x"]),
                    "y": float(kf_json["y"]),
                    "theta": float(kf_json["theta"]),
                    "xs": [float(v) for v in kf_json.get("xs", [])],
                    "ys": [float(v) for v in kf_json.get("ys", [])],
                })

            constraints = []
            for c_json in pg_data.get("constraints", []):
                constraints.append({
                    "from": int(c_json["from"]),
                    "to": int(c_json["to"]),
                    "dx": float(c_json["dx"]),
                    "dy": float(c_json["dy"]),
                    "dtheta": float(c_json["dtheta"]),
                })

            result = {
                "map_id": map_id,
                "type": "aimee_nav",
                "resolution": map_data["resolution"],
                "width": map_data["width"],
                "height": map_data["height"],
                "origin_x": map_data["origin_x"],
                "origin_y": map_data["origin_y"],
                "grid_data": grid_list,
                "ekf": {
                    "x": ekf_data.get("x", 0.0),
                    "y": ekf_data.get("y", 0.0),
                    "theta": ekf_data.get("theta", 0.0),
                    "covariance": ekf_data.get("covariance", []),
                },
                "keyframes": keyframes,
                "constraints": constraints,
            }

            self._log_info(f"Native map loaded: {map_id}")
            return result
        except Exception as e:
            self._log_error(f"Failed to load native map {map_id}: {e}")
            return None

    def delete_map(self, map_id: str) -> bool:
        """Delete a map from the library."""
        try:
            map_dir = self._map_dir(map_id)
            if map_dir.exists():
                shutil.rmtree(map_dir)
            if map_id in self._manifest.get("maps", {}):
                del self._manifest["maps"][map_id]
                self._save_manifest()
            self._log_info(f"Map deleted: {map_id}")
            return True
        except Exception as e:
            self._log_error(f"Failed to delete map {map_id}: {e}")
            return False

    def import_ros_map(
        self,
        map_id: str,
        pgm_path: str,
        yaml_path: str,
        name: str = "",
        description: str = "",
    ) -> bool:
        """Import a ROS standard PGM+YAML map into the library."""
        try:
            pgm_path = Path(os.path.expanduser(pgm_path))
            yaml_path = Path(os.path.expanduser(yaml_path))

            if not pgm_path.exists():
                self._log_error(f"PGM file not found: {pgm_path}")
                return False
            if not yaml_path.exists():
                self._log_error(f"YAML file not found: {yaml_path}")
                return False

            with open(yaml_path, "r") as f:
                yaml_data = yaml.safe_load(f)

            resolution = float(yaml_data.get("resolution", 0.05))
            origin = yaml_data.get("origin", [0.0, 0.0, 0.0])
            negate = int(yaml_data.get("negate", 0))
            occupied_thresh = float(yaml_data.get("occupied_thresh", 0.65))
            free_thresh = float(yaml_data.get("free_thresh", 0.25))
            image_file = yaml_data.get("image", "")

            # Resolve image path relative to YAML if needed
            image_path = pgm_path
            if image_file and not Path(image_file).is_absolute():
                image_path = yaml_path.parent / image_file
                if not image_path.exists():
                    image_path = pgm_path

            # Read PGM
            grid_data, width, height = self._read_pgm(image_path)
            if grid_data is None:
                return False

            # Convert PGM values (0-255) to occupancy grid values (-1, 0, 100)
            occ_grid = self._pgm_to_occupancy(
                grid_data, negate, occupied_thresh, free_thresh
            )

            map_dir = self._map_dir(map_id)
            map_dir.mkdir(parents=True, exist_ok=True)

            # Store raw files for export
            shutil.copy2(image_path, map_dir / self.ROS_PGM_FILENAME)
            shutil.copy2(yaml_path, map_dir / self.ROS_YAML_FILENAME)

            # Also store as native grid for fast loading
            native_path = map_dir / self.NATIVE_MAP_FILENAME
            native_data = {
                "version": 1,
                "map": {
                    "resolution": resolution,
                    "width": width,
                    "height": height,
                    "origin_x": float(origin[0]),
                    "origin_y": float(origin[1]),
                    "grid_data_b64": base64.b64encode(
                        bytes(b if b >= 0 else 256 + b for b in occ_grid)
                    ).decode("ascii"),
                },
                "ekf": {"x": 0.0, "y": 0.0, "theta": 0.0, "covariance": []},
                "pose_graph": {"keyframes": [], "constraints": []},
            }
            with open(native_path, "w") as f:
                json.dump(native_data, f, indent=2)

            metadata = {
                "name": name or map_id,
                "description": description or f"Imported from {pgm_path.name}",
                "type": "ros_map",
                "created_at": self._now_iso(),
                "updated_at": self._now_iso(),
                "resolution": resolution,
                "width_m": width * resolution,
                "height_m": height * resolution,
                "origin_x": float(origin[0]),
                "origin_y": float(origin[1]),
                "has_waypoints": False,
            }
            meta_path = map_dir / self.METADATA_FILENAME
            with open(meta_path, "w") as f:
                json.dump(metadata, f, indent=2)

            self._manifest["maps"][map_id] = metadata
            self._save_manifest()

            self._log_info(f"ROS map imported: {map_id}")
            return True
        except Exception as e:
            self._log_error(f"Failed to import ROS map {map_id}: {e}")
            return False

    def export_ros_map(self, map_id: str, pgm_path: str, yaml_path: str) -> bool:
        """Export a map from the library to ROS standard PGM+YAML."""
        try:
            meta = self.get_map(map_id)
            if meta is None:
                self._log_error(f"Map not found: {map_id}")
                return False

            map_dir = self._map_dir(map_id)
            pgm_out = Path(os.path.expanduser(pgm_path))
            yaml_out = Path(os.path.expanduser(yaml_path))
            pgm_out.parent.mkdir(parents=True, exist_ok=True)

            if meta.get("type") == "ros_map":
                # Direct copy of stored ROS files
                src_pgm = map_dir / self.ROS_PGM_FILENAME
                src_yaml = map_dir / self.ROS_YAML_FILENAME
                if src_pgm.exists():
                    shutil.copy2(src_pgm, pgm_out)
                if src_yaml.exists():
                    shutil.copy2(src_yaml, yaml_out)
                self._log_info(f"ROS map exported: {map_id} -> {pgm_out}")
                return True

            # Native map — need to generate PGM from grid data
            native_path = map_dir / self.NATIVE_MAP_FILENAME
            with open(native_path, "r") as f:
                data = json.load(f)

            map_data = data["map"]
            grid_bytes = base64.b64decode(map_data["grid_data_b64"])
            occ_grid = [b if b < 128 else b - 256 for b in grid_bytes]
            width = map_data["width"]
            height = map_data["height"]

            pgm_data = self._occupancy_to_pgm(occ_grid)
            self._write_pgm(pgm_out, pgm_data, width, height)

            yaml_data = {
                "image": pgm_out.name,
                "resolution": map_data["resolution"],
                "origin": [map_data["origin_x"], map_data["origin_y"], 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
            }
            with open(yaml_out, "w") as f:
                yaml.dump(yaml_data, f, default_flow_style=False)

            self._log_info(f"Native map exported to ROS format: {map_id} -> {pgm_out}")
            return True
        except Exception as e:
            self._log_error(f"Failed to export map {map_id}: {e}")
            return False

    def load_waypoints(self, map_id: str) -> Dict[str, Tuple[float, float, float]]:
        """Load waypoints for a map. Returns empty dict if none."""
        try:
            map_dir = self._map_dir(map_id)
            wp_path = map_dir / self.WAYPOINTS_FILENAME
            if not wp_path.exists():
                return {}
            with open(wp_path, "r") as f:
                data = yaml.safe_load(f)
            if not isinstance(data, dict):
                return {}
            waypoints = {}
            for name, pose in data.items():
                if isinstance(pose, dict):
                    waypoints[name] = (
                        float(pose.get("x", 0.0)),
                        float(pose.get("y", 0.0)),
                        float(pose.get("yaw", 0.0)),
                    )
                elif isinstance(pose, (list, tuple)) and len(pose) >= 2:
                    waypoints[name] = (
                        float(pose[0]),
                        float(pose[1]),
                        float(pose[2]) if len(pose) > 2 else 0.0,
                    )
            return waypoints
        except Exception as e:
            self._log_warn(f"Failed to load waypoints for {map_id}: {e}")
            return {}

    def save_waypoints(
        self, map_id: str, waypoints: Dict[str, Tuple[float, float, float]]
    ) -> bool:
        """Save waypoints for a map."""
        try:
            map_dir = self._map_dir(map_id)
            map_dir.mkdir(parents=True, exist_ok=True)
            self._write_waypoints(map_dir, waypoints)

            # Update manifest
            meta = self._manifest.get("maps", {}).get(map_id)
            if meta:
                meta["has_waypoints"] = bool(waypoints)
                meta["updated_at"] = self._now_iso()
                self._save_manifest()
            return True
        except Exception as e:
            self._log_error(f"Failed to save waypoints for {map_id}: {e}")
            return False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _serialize_native(self, global_map, ekf, pose_graph) -> Dict[str, Any]:
        """Serialize AimeeNav state to dict (mirror of aimee_nav_node.py logic)."""
        raw_grid = global_map.data()
        grid_bytes = bytes(b if b >= 0 else 256 + b for b in raw_grid)
        grid_b64 = base64.b64encode(grid_bytes).decode("ascii")

        P = ekf.covariance().tolist()

        kfs = pose_graph.keyframes()
        keyframes_json = []
        for kf in kfs:
            keyframes_json.append({
                "x": kf.x, "y": kf.y, "theta": kf.theta,
                "xs": list(kf.xs), "ys": list(kf.ys),
            })

        constraints = pose_graph.constraints()
        constraints_json = []
        for c in constraints:
            constraints_json.append({
                "from": getattr(c, "from"), "to": c.to,
                "dx": c.dx, "dy": c.dy, "dtheta": c.dtheta,
            })

        return {
            "version": 1,
            "map": {
                "resolution": global_map.resolution_m(),
                "width": global_map.width_cells(),
                "height": global_map.height_cells(),
                "origin_x": global_map.origin_x(),
                "origin_y": global_map.origin_y(),
                "grid_data_b64": grid_b64,
            },
            "ekf": {
                "x": ekf.x(),
                "y": ekf.y(),
                "theta": ekf.theta(),
                "covariance": P,
            },
            "pose_graph": {
                "keyframes": keyframes_json,
                "constraints": constraints_json,
            },
        }

    def _write_waypoints(
        self, map_dir: Path, waypoints: Dict[str, Tuple[float, float, float]]
    ) -> None:
        data = {}
        for name, (x, y, yaw) in waypoints.items():
            data[name] = {"x": x, "y": y, "yaw": yaw}
        wp_path = map_dir / self.WAYPOINTS_FILENAME
        with open(wp_path, "w") as f:
            yaml.dump(data, f, default_flow_style=False)

    @staticmethod
    def _now_iso() -> str:
        return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    # ------------------------------------------------------------------
    # PGM I/O
    # ------------------------------------------------------------------

    @staticmethod
    def _read_pgm(path: Path) -> Tuple[Optional[List[int]], int, int]:
        """Read a PGM file. Returns (data, width, height) or (None, 0, 0)."""
        with open(path, "rb") as f:
            header = f.readline().strip()
            if header not in (b"P5", b"P2"):
                raise ValueError(f"Unsupported PGM format: {header}")
            # Skip comments
            while True:
                line = f.readline()
                if not line.startswith(b"#"):
                    break
            dims = line.strip().split()
            width = int(dims[0])
            height = int(dims[1])
            maxval = int(f.readline().strip())
            if header == b"P5":
                data = list(f.read())
            else:
                data = [int(x) for x in f.read().split()]
        return data, width, height

    @staticmethod
    def _write_pgm(path: Path, data: List[int], width: int, height: int) -> None:
        """Write a binary PGM (P5) file."""
        with open(path, "wb") as f:
            f.write(f"P5\n{width} {height}\n255\n".encode())
            f.write(bytes(data))

    @staticmethod
    def _pgm_to_occupancy(
        pgm_data: List[int], negate: int, occupied_thresh: float, free_thresh: float
    ) -> List[int]:
        """Convert PGM pixel values to occupancy grid values (-1, 0, 100)."""
        occ = []
        for v in pgm_data:
            if negate:
                v = 255 - v
            p = v / 255.0
            if p > occupied_thresh:
                occ.append(100)
            elif p < free_thresh:
                occ.append(0)
            else:
                occ.append(-1)
        return occ

    @staticmethod
    def _occupancy_to_pgm(occ_grid: List[int]) -> List[int]:
        """Convert occupancy grid values to PGM pixel values."""
        pgm = []
        for v in occ_grid:
            if v == -1:
                pgm.append(205)  # Unknown -> light gray
            elif v >= 100:
                pgm.append(0)    # Occupied -> black
            else:
                pgm.append(254)  # Free -> white
        return pgm
