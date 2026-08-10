from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ouster_cluster',
            executable='ouster_cluster_node',
            name='ouster_cluster_node',
            output='screen',
            parameters=[{
                # front-of-pipeline cull (keeps the BEV stages real-time)
                'preprocess_max_range': 3.0,
                'preprocess_z_min': -0.5,
                'preprocess_z_max': 0.0,  # target sits below sensor height; cut
                                          # overhead clutter (false positives)
                'preprocess_voxel_leaf': 0.0,  # 0 = voxel downsample off (keep full plate density)
                # ray-elevation band cut (firmware cannot mask vertical beams):
                # keep -30..+15 deg -> drop floor cone + overhead clutter.
                # toggle live: ros2 param set /ouster_cluster_node elev_filter_enable false
                'elev_filter_enable': True,
                'elev_min_deg': -30.0,
                'elev_max_deg': 15.0,
                # BEV density map (intensity = per-cell count x dist^2, distance-invariant)
                # band-pass [min, max]: cut lone noise below, big face-on walls above
                # (values tuned live 2026-07-09 with the 14x10cm plate at ~1m)
                'bev_density_cell_size': 0.02,
                'bev_density_min': 7.0,  # kills weak flicker cells; plate cells
                                          # measure 30-300 so 3x margin remains
                                          # (measured 2026-07-13: 5->10 drops
                                          # surviving points 20.6k -> 18.5k)
                'bev_density_max': 0.0,  # 0 = cap off; a cap cut plate centers at
                                         # normal incidence (length gate handles walls)
                # verticality: a cell must span several occupied z-bins to survive
                # (floor = 1 bin -> dropped; a bin is occupied with >= 2 points)
                'bev_density_z_bin': 0.05,
                'bev_density_min_pts_per_bin': 2,
                'bev_density_min_z_bins': 2,
                # second grid shifted half a cell, OR-combined: a structure
                # split by a cell edge of one grid is whole in the other
                'bev_density_dual_grid': True,
                # isolated cells (no surviving cell in the 8-neighbourhood)
                # are noise — a real structure spans several adjacent cells
                'bev_density_drop_lonely': True,
                # BEV line segments: connected components of surviving cells that
                # are line-shaped (long along one axis, thin across it)
                'bev_line_min_cells': 5,
                'bev_line_max_width': 0.09,  # dual-grid fattens the plate to ~0.072;
                                             # 0.07 cut it by 2mm (measured 2026-07-14)
                'bev_line_min_length': 0.05,
                'bev_line_max_length': 0.25,  # 0 = no cap; target plate is 0.14
                'bev_line_z_gap': 0.10,  # neighbours connect only within this height gap
                'bev_line_diag_flank_gate': True,  # diagonal joins need BOTH
                                                   # flanking orthogonal cells to
                                                   # exist and pass — corner-only
                                                   # touches cannot leak in
                # point-level majority-touch connectivity: neighbours connect
                # when, in at least one direction, more than half of a cell's
                # points find a companion within touch_gap in z in the other
                # cell. Fixes the median rule cutting one object in two where
                # its visible height changes (occlusion), and one stray pair
                # cannot glue two structures (majority required).
                # Toggle live: ros2 param set ... bev_line_touch_connect false
                # (falls back to the z_med_gap / z_gap rules below)
                'bev_line_touch_connect': True,
                'bev_line_touch_gap': 0.015,
                # companion also needs this XY (horizontal) proximity; a
                # point 5cm+ away in XY within the same z-band no longer
                # counts as touching. 0 = disabled (z alone decides)
                'bev_line_touch_xy_gap': 0.03,
                # a direction also needs >=2 matched points: a 1-point cell
                # passes "majority" trivially (1/1) and chained the plate to
                # neighbours as a stepping stone (measured 2026-07-14: 21/50
                # frames fused into a 56-cell blob through such cells)
                'bev_line_touch_min_matched': 4,
                # fraction of a cell's points needing a companion (strictly
                # greater than) — 0.5 = old "more than half" rule; lower to
                # allow thinner overlap between adjacent cells
                'bev_line_touch_min_ratio': 0.6,
                'bev_line_z_med_gap': 0.04,  # >0: connect on MEDIAN cell height
                                             # instead of z-range overlap — one stray
                                             # point can't glue cells any more.
                                             # 0.0 = fall back to the range rule
                'bev_line_top_tol': 0.10,  # per-cell top heights must agree along the
                                           # line; dual-rescued edge cells push the
                                           # plate's top spread to ~0.09
                'bev_line_top_step': 0.07,  # split a component at persistent top-height steps
                # target plate shape check: segments matching 14cm(len) x 10cm(z-band)
                # are drawn GREEN on /ouster_cluster/bev_lines (ns "bev_plates")
                'bev_plate_width': 0.14,
                'bev_plate_height': 0.10,
                # independent tolerances: length drifts more than the z-band
                'bev_plate_tol_width': 0.05,   # +/- on BEV segment length
                'bev_plate_tol_height': 0.04,  # +/- on z-band height
                # the plate sheet is <= 3cm thick; anything wider across the
                # line is a box/blob (0 = disable)
                'bev_plate_thickness_max': 0.0,
                'bev_line_max_segments': 50,
                # point-level refit: robust line fit on raw points near the coarse
                # segment (MAD trimming + percentile endpoints) — sub-cell accuracy.
                # OFF: the tuned pipeline uses the coarse cell segments as-is
                # (bev_plate_thickness_max above is 0 to match — re-tune both
                # together when enabling; was previously disabled via the
                # min_points=100000 workaround)
                'bev_refine_enable': False,
                'bev_refine_band': 0.02,
                'bev_refine_mad_k': 2.5,
                'bev_refine_end_percentile': 0.05,
                'bev_refine_min_points': 10,
                # one-material gate: points supporting a segment must have
                # uniform scalar (p90-p10 <= spread) and median in [min, max].
                # Uses whatever scalar_field carries (reflectivity/signal) —
                # switch live to A/B: ros2 param set ... scalar_field signal
                'bev_refl_enable': False,  # off: the target plate's bottom strip
                                           # has very different reflectivity, so
                                           # the one-material check rejects it
                'bev_refl_band': 0.05,
                'bev_refl_spread_max': 255.0,  # if re-enabled, keep uniformity
                                               # loose for the two-tone plate
                'bev_refl_min': 5.0,   # cable-splash ghosts measure ~1 uniformly
                'bev_refl_max': 255.0,
                'bev_refl_min_points': 10,
                # temporal segment filter: publish after N consecutive frames,
                # hold a confirmed segment through short dropouts
                'bev_seg_match_distance': 0.15,
                'bev_seg_confirm_frames': 10,
                'bev_seg_max_missed': 2,
                # final precision stage: seed-grown clustering on each GREEN
                # segment's RAW points (z-cropped against the floor) + shape
                # check -> EMA-tracked boxes on /ouster_cluster/boxes. Recovers
                # the full plate when the density gates shortened the segment.
                'plate_recover_enable': True,
                'plate_recover_xy_half': 0.12,        # plate half-width + margin
                'plate_recover_z_margin_below': 0.0,  # floor guard: keep tight
                'plate_recover_z_margin_above': 0.02,
                'plate_recover_tolerance': 0.03,
                'plate_recover_min_points': 10,
                # final shape trim on the picked cluster: strays chained in
                # within tolerance but off the cluster's mean+covariance shape
                # are dropped (squared Mahalanobis, chi^2 dof=3 95%; 0 = off)
                'plate_recover_maha_threshold': 7.815,
                # accumulate the last N preprocessed frames for the recovery
                # input only: at range one frame's scan rings are too sparse to
                # cluster at a tight tolerance, but successive frames interleave
                # and fill the gaps. 1 = off. Assumes a static target (a moving
                # one smears). 0.1s latency per extra frame at 10Hz.
                'plate_recover_accum_frames': 1,
                'plate_thickness_max': 0.03,
                'plate_width': 0.14,
                'plate_height': 0.10,
                'plate_size_tol': 0.03,
                'plate_min_points': 10,
                'track_match_distance': 0.5,
                'track_pos_alpha': 0.5,
                'track_vel_alpha': 0.5,
                'track_yaw_alpha': 0.2,
                'track_confirm_frames': 2,
                'track_max_missed': 3,
                'track_max_dt': 0.3,
                'input_topic': '/ouster/points',
                # driver runs with point_type:=original (proc_mask:=IMU|PCL) —
                # Point-LIO needs the per-point `t` field, which xyzi lacks.
                # `intensity` (signal, float32) is still present, so behavior is
                # unchanged; `reflectivity` (uint16) is also available now for
                # A/B: ros2 param set ... scalar_field reflectivity.
                # Only used by the refl gate (bev_refl_enable), currently off.
                'scalar_field': 'intensity',
            }],
        ),
    ])
