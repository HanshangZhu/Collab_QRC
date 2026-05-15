import numpy as np

from trav_cost_filters.occupancy_conversion import (
    apply_slope_verified_ramp_override,
    stamp_free_disk,
    traversability_to_occupancy,
)


def test_traversability_thresholds_keep_moderate_ramp_scores_free():
    trav = np.array(
        [
            [np.nan, 0.0, 0.14],
            [0.30, 0.53, 1.0],
        ],
        dtype=np.float32,
    )

    occ = traversability_to_occupancy(
        trav,
        free_threshold=0.30,
        lethal_threshold=0.15,
    )

    assert occ.tolist() == [
        [-1, 100, 100],
        [0, 0, 0],
    ]


def test_robot_footprint_seed_connects_unknown_start_without_clearing_walls():
    occ = np.full((9, 9), -1, dtype=np.int8)
    occ[4, 6] = 100

    changed = stamp_free_disk(
        occ,
        origin_x=-0.4,
        origin_y=-0.4,
        resolution=0.1,
        center_x=0.0,
        center_y=0.0,
        radius_m=0.21,
    )

    assert changed > 0
    assert occ[4, 4] == 0
    assert occ[4, 6] == 100


def test_slope_verified_ramp_override_clears_continuous_ramp_not_wall():
    occ = np.array([[100, 100, 100, 0]], dtype=np.int8)
    slope = np.array([[0.24, 1.20, 0.24, 0.02]], dtype=np.float32)
    step_residual = np.array([[0.01, 0.01, 0.30, 0.0]], dtype=np.float32)

    changed = apply_slope_verified_ramp_override(
        occ,
        slope=slope,
        step_residual=step_residual,
        min_slope_rad=0.14,
        max_slope_rad=0.52,
        max_step_residual_m=0.06,
    )

    assert changed == 1
    assert occ.tolist() == [[0, 100, 100, 0]]
