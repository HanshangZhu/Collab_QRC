#!/usr/bin/env python3
"""Unit tests for exploration benchmark helper scripts.

These tests intentionally avoid ROS runtime dependencies; they validate the
deterministic file-generation and aggregation pieces that are safe to exercise
in CI or from a plain shell.
"""

from __future__ import annotations

import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BENCH_DIR = ROOT / "scripts" / "bench"
RUNTIME_DIR = ROOT / "scripts" / "runtime"
if str(BENCH_DIR) not in sys.path:
    sys.path.insert(0, str(BENCH_DIR))
if str(RUNTIME_DIR) not in sys.path:
    sys.path.insert(0, str(RUNTIME_DIR))


class GeneratedMazeTests(unittest.TestCase):
    def test_generation_is_deterministic_and_preserves_robot_bodies(self) -> None:
        from generate_exploration_mazes import generate_maze_scene

        template = ROOT / "src" / "go2w" / "go2_gazebo_sim" / "mujoco" / "demo3_mixed.xml"
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            first = generate_maze_scene(
                template_path=template,
                output_dir=tmp_path,
                variant="rooms",
                seed=101,
            )
            second = generate_maze_scene(
                template_path=template,
                output_dir=tmp_path,
                variant="rooms",
                seed=101,
            )

            xml_text = first.scene_path.read_text()
            self.assertEqual(first.scene_path.read_text(), second.scene_path.read_text())
            self.assertIn('<mujoco model="demo3_mixed_rooms_seed101">', xml_text)
            self.assertIn('<body name="base_link"', xml_text)
            self.assertIn('<body name="b_base_link"', xml_text)
            self.assertIn("generated_rooms_", xml_text)
            self.assertAlmostEqual(first.metadata["scene_area_m2"], 384.0)
            self.assertEqual(first.metadata["seed"], 101)
            self.assertEqual(first.metadata["variant"], "rooms")
            self.assertTrue(first.metadata_path.exists())

    def test_generated_layouts_keep_robot_passable_clearances(self) -> None:
        from generate_exploration_mazes import generate_maze_scene

        template = ROOT / "src" / "go2w" / "go2_gazebo_sim" / "mujoco" / "demo3_mixed.xml"
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            for variant, seed in (("rooms", 101), ("corridors", 202)):
                generated = generate_maze_scene(
                    template_path=template,
                    output_dir=tmp_path,
                    variant=variant,
                    seed=seed,
                )
                xml_text = generated.scene_path.read_text()
                self.assertGreaterEqual(generated.metadata["minimum_gate_width_m"], 1.6)
                self.assertGreaterEqual(generated.metadata["target_corridor_width_m"], 1.8)
                self.assertIn("clean_", xml_text)
                self.assertNotIn("0.9-1.2m doors", xml_text)


class BenchmarkAggregationTests(unittest.TestCase):
    def test_summary_uses_global_csv_and_per_robot_json(self) -> None:
        from aggregate_exploration_benchmark import collect_trial_records, summarise_records

        with tempfile.TemporaryDirectory() as tmp:
            trial_dir = Path(tmp) / "demo3_mixed" / "cfpa2" / "trial_01"
            trial_dir.mkdir(parents=True)
            for ns, area, distance in (
                ("robot_a", 120.0, 31.0),
                ("robot_b", 95.0, 25.0),
            ):
                (trial_dir / f"{ns}.json").write_text(json.dumps({
                    "duration_target_sec": 600.0,
                    "elapsed_sec": 600.0,
                    "started_at_unix": 1000.0,
                    "ended_at_unix": 1600.0,
                    "coverage": {
                        "explored_area_m2": area,
                        "coverage_ratio_of_scene": area / 384.0,
                    },
                    "progress": {
                        "distance_travelled_m": distance,
                        "tipped_over": ns == "robot_b",
                        "degraded_tilt": {"entry_count": 1 if ns == "robot_b" else 0},
                    },
                    "slam": {
                        "trans_error_mean_m": 0.1 if ns == "robot_a" else 0.2,
                        "trans_error_final_m": 0.15 if ns == "robot_a" else 0.25,
                    },
                    "safety": {
                        "wall_contact_count": 0,
                        "obstacle_contact_count": 2 if ns == "robot_b" else 0,
                        "ever_touched": ns == "robot_b",
                    },
                    "outcome": "completed",
                }))

            with (trial_dir / "metrics.csv").open("w", newline="") as f:
                writer = csv.DictWriter(
                    f,
                    fieldnames=[
                        "t_wall",
                        "t_sim",
                        "global_explored_area_m2",
                        "global_coverage_ratio",
                        "overlap_pct",
                        "robot_a_trajectory_m",
                        "robot_a_coverage_area_m2",
                        "robot_b_trajectory_m",
                        "robot_b_coverage_area_m2",
                    ],
                )
                writer.writeheader()
                writer.writerow({
                    "t_wall": "0.0",
                    "t_sim": "10.0",
                    "global_explored_area_m2": "96.0",
                    "global_coverage_ratio": "0.25",
                    "overlap_pct": "10.0",
                    "robot_a_trajectory_m": "5.0",
                    "robot_a_coverage_area_m2": "80.0",
                    "robot_b_trajectory_m": "4.0",
                    "robot_b_coverage_area_m2": "70.0",
                })
                writer.writerow({
                    "t_wall": "110.0",
                    "t_sim": "130.0",
                    "global_explored_area_m2": "120.0",
                    "global_coverage_ratio": "0.3125",
                    "overlap_pct": "11.0",
                    "robot_a_trajectory_m": "12.0",
                    "robot_a_coverage_area_m2": "105.0",
                    "robot_b_trajectory_m": "10.0",
                    "robot_b_coverage_area_m2": "90.0",
                })
                writer.writerow({
                    "t_wall": "230.0",
                    "t_sim": "250.0",
                    "global_explored_area_m2": "160.0",
                    "global_coverage_ratio": "0.4166666667",
                    "overlap_pct": "12.0",
                    "robot_a_trajectory_m": "22.0",
                    "robot_a_coverage_area_m2": "130.0",
                    "robot_b_trajectory_m": "19.0",
                    "robot_b_coverage_area_m2": "120.0",
                })
                writer.writerow({
                    "t_wall": "300.0",
                    "t_sim": "310.0",
                    "global_explored_area_m2": "190.0",
                    "global_coverage_ratio": "0.4947916667",
                    "overlap_pct": "13.0",
                    "robot_a_trajectory_m": "31.0",
                    "robot_a_coverage_area_m2": "120.0",
                    "robot_b_trajectory_m": "25.0",
                    "robot_b_coverage_area_m2": "95.0",
                })
            (trial_dir / "exploration_events_demo.log").write_text(
                "[00:00:01.000 +----ms] PLAN_RETURNED: ns=robot_a planner=ComputePathToPose t=20ms ok\n"
                "[00:00:02.000 +1000ms] PLAN_FAILED: ns=robot_b planner=ComputePathToPose t=40ms fail\n"
            )

            records = collect_trial_records(Path(tmp))
            summary = summarise_records(records)
            key = ("demo3_mixed", "cfpa2")
            self.assertIn(key, summary)
            self.assertEqual(summary[key]["trial_count"], 1)
            self.assertAlmostEqual(summary[key]["global_explored_area_m2_mean"], 190.0)
            self.assertAlmostEqual(summary[key]["global_coverage_ratio_mean"], 0.4947916667)
            self.assertAlmostEqual(summary[key]["robot_a_distance_m_mean"], 31.0)
            self.assertAlmostEqual(summary[key]["robot_b_distance_m_mean"], 25.0)
            self.assertAlmostEqual(summary[key]["real_time_factor_mean"], 1.0)
            self.assertAlmostEqual(summary[key]["time_to_coverage_25pct_sec_mean"], 0.0)
            self.assertAlmostEqual(summary[key]["coverage_50pct_reach_rate"], 0.0)
            self.assertAlmostEqual(summary[key]["robot_b_obstacle_contacts_mean"], 2.0)
            self.assertAlmostEqual(summary[key]["robot_b_ever_touched_rate"], 1.0)
            self.assertAlmostEqual(summary[key]["robot_b_tipped_over_rate"], 1.0)
            self.assertAlmostEqual(summary[key]["robot_b_degraded_tilt_rate"], 1.0)
            self.assertAlmostEqual(summary[key]["robot_a_slam_trans_error_mean_m_mean"], 0.1)
            self.assertAlmostEqual(summary[key]["nav2_plan_ms_mean"], 20.0)
            self.assertAlmostEqual(summary[key]["nav2_plan_success_rate"], 0.5)
            self.assertAlmostEqual(summary[key]["at_120s_global_explored_area_m2_mean"], 120.0)
            self.assertAlmostEqual(summary[key]["at_120s_global_coverage_ratio_mean"], 0.3125)
            self.assertAlmostEqual(summary[key]["at_120s_robot_a_distance_m_mean"], 12.0)
            self.assertAlmostEqual(summary[key]["at_120s_robot_a_explored_area_m2_mean"], 105.0)
            self.assertAlmostEqual(summary[key]["at_240s_global_explored_area_m2_mean"], 160.0)
            self.assertAlmostEqual(summary[key]["at_240s_robot_b_distance_m_mean"], 19.0)
            self.assertAlmostEqual(summary[key]["at_240s_robot_b_explored_area_m2_mean"], 120.0)
            self.assertIsNone(summary[key]["at_360s_global_explored_area_m2_mean"])
            self.assertIsNone(summary[key]["at_480s_robot_a_distance_m_mean"])


class BenchmarkValidationTests(unittest.TestCase):
    def _write_valid_trial(self, trial_dir: Path) -> None:
        trial_dir.mkdir(parents=True)
        (trial_dir / "exit_code.txt").write_text("0\n")
        for ns, distance in (("robot_a", 12.0), ("robot_b", 10.0)):
            (trial_dir / f"{ns}.json").write_text(json.dumps({
                "outcome": "completed",
                "progress": {"distance_travelled_m": distance},
                "coverage": {"explored_area_m2": 100.0},
            }))
        with (trial_dir / "exploration_test.csv").open("w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "t_sim",
                    "global_explored_area_m2",
                    "global_coverage_ratio",
                    "robot_a_trajectory_m",
                    "robot_a_coverage_area_m2",
                    "robot_b_trajectory_m",
                    "robot_b_coverage_area_m2",
                ],
            )
            writer.writeheader()
            writer.writerow({
                "t_sim": "10.0",
                "global_explored_area_m2": "50.0",
                "global_coverage_ratio": "0.13",
                "robot_a_trajectory_m": "1.0",
                "robot_a_coverage_area_m2": "45.0",
                "robot_b_trajectory_m": "1.0",
                "robot_b_coverage_area_m2": "42.0",
            })
            writer.writerow({
                "t_sim": "130.0",
                "global_explored_area_m2": "120.0",
                "global_coverage_ratio": "0.31",
                "robot_a_trajectory_m": "12.0",
                "robot_a_coverage_area_m2": "100.0",
                "robot_b_trajectory_m": "10.0",
                "robot_b_coverage_area_m2": "92.0",
            })
        (trial_dir / "exploration_events_test.log").write_text(
            "[00:00:01.000 +----ms] PLAN_RETURNED: ns=robot_a planner=ComputePathToPose t=20ms ok\n"
        )

    def test_trial_validation_accepts_required_benchmark_outputs(self) -> None:
        from validate_exploration_trial import validate_trial

        with tempfile.TemporaryDirectory() as tmp:
            trial_dir = Path(tmp) / "trial_01"
            self._write_valid_trial(trial_dir)
            result = validate_trial(trial_dir, duration_sec=150.0)
            self.assertTrue(result["valid"], result)
            self.assertTrue((trial_dir / "validation.json").exists())

    def test_trial_validation_rejects_missing_checkpoint_sample(self) -> None:
        from validate_exploration_trial import validate_trial

        with tempfile.TemporaryDirectory() as tmp:
            trial_dir = Path(tmp) / "trial_01"
            self._write_valid_trial(trial_dir)
            text = (trial_dir / "exploration_test.csv").read_text()
            (trial_dir / "exploration_test.csv").write_text(text.replace("130.0", "50.0"))
            result = validate_trial(trial_dir, duration_sec=150.0)
            self.assertFalse(result["valid"])
            self.assertIn("120s checkpoint", "\n".join(result["errors"]))


class MTARECommonExecutorTests(unittest.TestCase):
    def test_assigns_distinct_frontiers_from_map_and_robot_positions(self) -> None:
        from mtare_common_executor_core import assign_frontiers, extract_frontier_clusters

        width = 12
        height = 6
        data = [-1] * (width * height)
        for y in range(1, 5):
            for x in range(1, 11):
                data[y * width + x] = 0
        # Create two separated frontier-rich pockets by adding known free
        # islands next to unknown space at opposite ends.
        clusters = extract_frontier_clusters(
            data=data,
            width=width,
            height=height,
            resolution=1.0,
            origin_x=0.0,
            origin_y=0.0,
            min_cluster_size=2,
        )

        assignments = assign_frontiers(
            clusters=clusters,
            robot_positions={"robot_a": (2.0, 2.0), "robot_b": (9.0, 3.0)},
            previous_goals={},
            min_peer_goal_separation=2.0,
        )

        self.assertEqual(set(assignments), {"robot_a", "robot_b"})
        ax, ay = assignments["robot_a"]
        bx, by = assignments["robot_b"]
        self.assertGreater(((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5, 2.0)


class BaselineWrapperTests(unittest.TestCase):
    def test_benchmark_runner_defaults_to_formal_three_planner_matrix(self) -> None:
        runner = ROOT / "scripts" / "bench" / "benchmark_exploration_planners.sh"
        text = runner.read_text()
        self.assertIn('NUM_TRIALS="${NUM_TRIALS:-3}"', text)
        self.assertIn('PLANNERS="${PLANNERS:-cfpa2 gbplanner2 mtare}"', text)
        self.assertIn("Allowed: cfpa2 gbplanner2 mtare", text)
        self.assertIn("gbplanner3 is intentionally excluded", text)

    def test_benchmark_runner_requires_vendor_mtare_for_formal_runs(self) -> None:
        runner = ROOT / "scripts" / "bench" / "benchmark_exploration_planners.sh"
        text = runner.read_text()
        self.assertIn('ALLOW_MTARE_FALLBACK="${ALLOW_MTARE_FALLBACK:-false}"', text)
        self.assertIn("find_mtare_vendor_executable", text)
        self.assertIn("mtare benchmark requires a built vendor TARE executable", text)
        self.assertIn("ALLOW_MTARE_FALLBACK=true for non-formal fallback debug", text)
        self.assertIn('"mtare_vendor_executable"', text)

    def test_gbplanner_dual_wrapper_declares_required_topics(self) -> None:
        wrapper = ROOT / "scripts" / "sim" / "gbplanner3_mujoco" / "launch_dual_common_executor.sh"
        compose = ROOT / "scripts" / "sim" / "gbplanner3_mujoco" / "compose" / "docker-compose.collab_qrc_dual.yml"
        text = wrapper.read_text() + "\n" + compose.read_text()
        self.assertIn("/robot_a/command/trajectory", text)
        self.assertIn("/robot_b/command/trajectory", text)
        self.assertIn("ROS_MASTER_URI=http://localhost:11311", text)
        self.assertIn("ROS_MASTER_URI=http://localhost:11312", text)
        self.assertIn("GBPLANNER_VERSION", text)
        self.assertIn("gbplanner2_config.yaml", text)
        self.assertIn("gbplanner_service_path_executor.py", text)
        self.assertIn("service_path", text)

    def test_cfpa2_benchmark_launch_uses_live_robot_maps(self) -> None:
        launch = ROOT / "src" / "go2w" / "go2_gazebo_sim" / "launch" / "nav_test_mujoco_fastlio_mixed.launch.py"
        text = launch.read_text()
        self.assertIn('"planning_map_topic_suffix": "/map"', text)

    def test_cfpa2_declares_planner_mode_before_runtime_use(self) -> None:
        coordinator = (
            ROOT
            / "src"
            / "collaborative_exploration"
            / "cfpa2_collaborative_autonomy"
            / "cfpa2_collaborative_autonomy"
            / "cfpa2_coordinator_node.py"
        )
        config = (
            ROOT
            / "src"
            / "collaborative_exploration"
            / "cfpa2_collaborative_autonomy"
            / "config"
            / "cfpa2_coordinator.yaml"
        )
        text = coordinator.read_text()
        declare_idx = text.index('declare_parameter("cfpa2_planner_mode", "greedy")')
        assign_idx = text.index("self.cfpa2_planner_mode = _planner_mode")
        use_idx = text.index('if self.cfpa2_planner_mode == "tsp_topk"')
        self.assertLess(declare_idx, assign_idx)
        self.assertLess(assign_idx, use_idx)
        self.assertIn('not in ("greedy", "tsp_topk")', text)
        self.assertIn("cfpa2_planner_mode: greedy", config.read_text())

    def test_mixed_benchmark_launch_uses_available_cpu_mppi_plugin(self) -> None:
        launch = ROOT / "src" / "go2w" / "go2_gazebo_sim" / "launch" / "nav_test_mujoco_fastlio_mixed.launch.py"
        overlay = ROOT / "src" / "go2w" / "go2w_config" / "config" / "nav" / "nav2_cpu_mppi_overlay_sim.yaml"
        self.assertTrue(overlay.exists())
        self.assertIn("nav2_cpu_mppi_overlay_sim.yaml", launch.read_text())
        self.assertIn('plugin: "nav2_mppi_controller::MPPIController"', overlay.read_text())
        self.assertIn("use_cuda: false", overlay.read_text())

    def test_mtare_vendor_launch_requires_real_executable(self) -> None:
        launch = ROOT / "src" / "go2w" / "go2_gazebo_sim" / "launch" / "nav_test_mujoco_fastlio_mixed.launch.py"
        text = launch.read_text()
        self.assertIn("mtare_executable_candidates", text)
        self.assertIn("os.path.isfile(path) and os.access(path, os.X_OK)", text)
        self.assertIn("exec {shlex.quote(mtare_executable)}", text)
        self.assertIn("Vendor TARE executable was not found", text)


if __name__ == "__main__":
    unittest.main()
