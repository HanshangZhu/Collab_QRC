#!/usr/bin/env python3
"""Fail fast if the ROS 2 workspace has duplicate package names or skipped COLCON_IGNORE paths.

Run from repo root (same layout as colcon):  python3 scripts/ops/workspace_colcon_audit.py

Colcon discovers packages under ./src; this script mirrors that and skips any directory
that contains COLCON_IGNORE (including ancestors — if src/vendor/foo/COLCON_IGNORE exists,
everything under foo/ is excluded).
"""
from __future__ import annotations

import re
import sys
from pathlib import Path
from xml.etree import ElementTree as ET


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    src = root / "src"
    if not src.is_dir():
        print(f"ERROR: expected {src}", file=sys.stderr)
        return 2

    # Directories that must not be treated as colcon roots (marker files).
    ignored_roots: set[Path] = set()
    for marker in src.rglob("COLCON_IGNORE"):
        if marker.is_file():
            ignored_roots.add(marker.parent.resolve())

    def under_ignored(dirpath: Path) -> bool:
        rp = dirpath.resolve()
        for ign in ignored_roots:
            try:
                rp.relative_to(ign)
                return True
            except ValueError:
                continue
        return False

    # Duplicate <name> among discoverable packages.
    by_name: dict[str, list[Path]] = {}
    for pkg_xml in src.rglob("package.xml"):
        parent = pkg_xml.parent
        if under_ignored(parent):
            continue
        try:
            tree = ET.parse(pkg_xml)
            name_el = tree.getroot().find("name")
            if name_el is None or not name_el.text:
                continue
            pkg_name = name_el.text.strip()
        except ET.ParseError as e:
            print(f"WARN: parse {pkg_xml}: {e}", file=sys.stderr)
            continue
        by_name.setdefault(pkg_name, []).append(pkg_xml)

    dup = {k: v for k, v in by_name.items() if len(v) > 1}
    if dup:
        print("DUPLICATE package.xml <name> entries (colcon will fail or pick undefined winner):\n")
        for name in sorted(dup):
            print(f"  {name}:")
            for p in dup[name]:
                print(f"    - {p.relative_to(root)}")
        print()
        ok = False
    else:
        print("OK: no duplicate package names among non-ignored paths.\n")
        ok = True

    # Workspace-only find_package(REQUIRED) targets that must appear in package.xml <depend>.
    workspace_pkgs = set(by_name.keys())
    cmake_pat = re.compile(r"find_package\s*\(\s*([a-zA-Z0-9_]+)\s+([^)]*)\)", re.MULTILINE | re.DOTALL)

    ros_core = frozenset(
        {
            "ament_cmake",
            "ament_cmake_auto",
            "ament_cmake_ros",
            "ament_cmake_python",
            "ament_index_cpp",
            "ament_lint_auto",
            "ament_lint_common",
            "launch_testing_ament_cmake",
            "ament_cmake_gtest",
            "rclcpp",
            "rclcpp_components",
            "rclpy",
            "rosidl_default_generators",
            "rosidl_default_runtime",
            "builtin_interfaces",
            "std_msgs",
            "geometry_msgs",
            "sensor_msgs",
            "nav_msgs",
            "visualization_msgs",
            "std_srvs",
            "tf2",
            "tf2_ros",
            "tf2_geometry_msgs",
            "tf2_sensor_msgs",
            "tf2_msgs",
            "message_filters",
            "pcl_conversions",
            "pcl_ros",
            "pcl_msgs",
            "OpenCV",
            "Eigen3",
            "PCL",
            "Boost",
            "PythonLibs",
            "OpenMP",
            "Doxygen",
            "PkgConfig",
            "catkin",
            "ortools",
            "GTSAM",
            "image_geometry",
            "map_msgs",
            "apr",
            "urdf",
            "rosbag2_cpp",
            "srv_msgs",
            "common_interfaces",
            "trajectory_msgs",
        }
    )

    missing_dep: list[tuple[Path, str, str]] = []
    for pkg_xml in src.rglob("package.xml"):
        parent = pkg_xml.parent
        if under_ignored(parent):
            continue
        cmakelists = parent / "CMakeLists.txt"
        if not cmakelists.is_file():
            continue
        body = cmakelists.read_text(encoding="utf-8", errors="replace")
        pkg_tree = ET.parse(pkg_xml)
        root_el = pkg_tree.getroot()
        declared = set()
        for tag in (
            "depend",
            "build_depend",
            "build_export_depend",
            "exec_depend",
            "buildtool_depend",
            "test_depend",
        ):
            for el in root_el.findall(tag):
                if el.text:
                    declared.add(el.text.strip())

        for m in cmake_pat.finditer(body):
            dep = m.group(1)
            rest = m.group(2)
            if "QUIET" in rest:
                continue
            if "REQUIRED" not in rest:
                continue
            if dep in ros_core:
                continue
            if dep not in workspace_pkgs:
                continue
            if dep not in declared:
                missing_dep.append((pkg_xml, parent.name, dep))

    if missing_dep:
        ok = False
        print("CMakeLists.txt find_package(... REQUIRED) without matching package.xml dependency:\n")
        for pkg_xml, folder, dep in sorted(missing_dep, key=lambda x: (x[2], str(x[0]))):
            print(f"  {folder}: find_package({dep}) — add <depend>{dep}</depend> to {pkg_xml.relative_to(root)}")
        print()

    if ok:
        print("workspace_colcon_audit: PASS")
        return 0
    print("workspace_colcon_audit: FAIL — fix duplicates and/or package.xml deps.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
