import numpy as np

from trav_cost_filters.occupancy_conversion import (
    apply_rectangular_workspace_mask,
    apply_slope_verified_ramp_override,
    project_rolling_grid_to_fixed_grid,
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


def test_rolling_origin_projects_same_world_cell_to_fixed_grid_index():
    fixed = np.full((8, 8), -1, dtype=np.int8)
    hits = np.zeros_like(fixed, dtype=np.int16)

    first = np.full((4, 4), -1, dtype=np.int8)
    first[2, 2] = 100  # world cell centred near (0.05, 0.05).
    project_rolling_grid_to_fixed_grid(
        first,
        fixed,
        hits,
        rolling_origin_x=-0.2,
        rolling_origin_y=-0.2,
        fixed_origin_x=-0.4,
        fixed_origin_y=-0.4,
        resolution=0.1,
        occupied_confirm_hits=1,
    )

    second = np.full((4, 4), -1, dtype=np.int8)
    second[1, 1] = 100  # Same world cell after rolling window shifted +0.1m.
    project_rolling_grid_to_fixed_grid(
        second,
        fixed,
        hits,
        rolling_origin_x=-0.1,
        rolling_origin_y=-0.1,
        fixed_origin_x=-0.4,
        fixed_origin_y=-0.4,
        resolution=0.1,
        occupied_confirm_hits=1,
    )

    assert np.argwhere(fixed == 100).tolist() == [[4, 4]]


def test_unknown_rolling_cells_do_not_erase_fixed_world_history():
    fixed = np.full((5, 5), -1, dtype=np.int8)
    hits = np.zeros_like(fixed, dtype=np.int16)
    fixed[2, 2] = 0

    rolling = np.full((5, 5), -1, dtype=np.int8)
    changed = project_rolling_grid_to_fixed_grid(
        rolling,
        fixed,
        hits,
        rolling_origin_x=-0.25,
        rolling_origin_y=-0.25,
        fixed_origin_x=-0.25,
        fixed_origin_y=-0.25,
        resolution=0.1,
    )

    assert changed == 0
    assert fixed[2, 2] == 0


def test_temporal_projection_filters_single_frame_obstacle_speckle():
    fixed = np.full((3, 3), -1, dtype=np.int8)
    hits = np.zeros_like(fixed, dtype=np.int16)

    speckle = np.full((3, 3), -1, dtype=np.int8)
    speckle[1, 1] = 100
    project_rolling_grid_to_fixed_grid(
        speckle,
        fixed,
        hits,
        rolling_origin_x=0.0,
        rolling_origin_y=0.0,
        fixed_origin_x=0.0,
        fixed_origin_y=0.0,
        resolution=0.1,
        occupied_confirm_hits=2,
    )

    assert fixed[1, 1] == -1
    assert hits[1, 1] == 1

    project_rolling_grid_to_fixed_grid(
        speckle,
        fixed,
        hits,
        rolling_origin_x=0.0,
        rolling_origin_y=0.0,
        fixed_origin_x=0.0,
        fixed_origin_y=0.0,
        resolution=0.1,
        occupied_confirm_hits=2,
    )

    assert fixed[1, 1] == 100
    assert hits[1, 1] == 2


def test_rectangular_workspace_mask_keeps_square_border_stable():
    occ = np.zeros((6, 6), dtype=np.int8)

    changed = apply_rectangular_workspace_mask(
        occ,
        origin_x=0.0,
        origin_y=0.0,
        resolution=1.0,
        min_x=1.0,
        max_x=5.0,
        min_y=1.0,
        max_y=5.0,
        wall_thickness_m=1.0,
    )

    assert changed > 0
    assert occ[0, 0] == -1
    assert occ[1, 1] == 100
    assert occ[3, 3] == 0
